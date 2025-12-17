import socket
import time
import threading
from collections import deque

class ManipulatorController:
    def __init__(self, host="192.168.42.241", port=8888):
        """Инициализация контроллера"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setblocking(False)
        self.udp_socket.settimeout(1.0)  # 1 секунда таймаут
        self.target_address = (host, port)
        self.udp_socket.bind(("0.0.0.0", port))
        
        # Параметры для обработки обратной связи
        self.last_three_ip = host.split('.')[-1]  # Последние 3 цифры IP
        self.feedback_buffer = deque(maxlen=10)  # Буфер для хранения последних сообщений
        self.is_moving = False  # Флаг движения манипулятора
        self.feedback_received = False  # Флаг получения обратной связи
        self.lock = threading.Lock()
        
        # Запуск потока для приема обратной связи
        self.listening = True
        self.feedback_thread = threading.Thread(target=self._listen_feedback)
        self.feedback_thread.daemon = True
        self.feedback_thread.start()
    
    def _listen_feedback(self):
        """Поток для приема обратной связи"""
        while self.listening:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                message = data.decode('utf-8', errors='ignore')
                self.feedback_buffer.append(message)
                self._parse_feedback(message)    
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving feedback: {e}")
    
    def _parse_feedback(self, message):
        """Парсинг сообщения обратной связи"""
        try:
            # Ищем сообщение вида: M:IP:STATUS:...
            parts = message.split()
            for part in parts:
                if part.startswith('M:'):
                    m_parts = part.split(':')
                    if len(m_parts) >= 3:
                        ip_part = m_parts[1]  # Часть с IP
                        status = m_parts[2]   # Статус движения (0 - остановлен, 1 - движется)
                        # Проверяем совпадение последних 3 цифр IP
                        if ip_part.endswith(self.last_three_ip):
                            with self.lock:
                                self.is_moving = (status == "1")
                                self.feedback_received = True
                            #print(f"Feedback: IP match, moving={self.is_moving}")
                            return
        except Exception as e:
            print(f"Error parsing feedback: {e}")
    
    def wait_for_completion(self, timeout=10):
        """Ожидание завершения движения манипулятора"""
        start_time = time.time()
        last_feedback_time = time.time()
        movement_started = False
        
        while time.time() - start_time < timeout:
            with self.lock:
                current_moving = self.is_moving
                current_feedback = self.feedback_received
            
            # Если получили обратную связь о начале движения
            if current_feedback and current_moving:
                movement_started = True
                #print("Movement started, waiting for completion...")
            
            # Если движение началось, обратная связь получена и флаг текущего движения манипулятора сброшен
            if movement_started and current_feedback and not current_moving:
                print("Movement completed")
                with self.lock:
                    self.feedback_received = False
                return True
            
            # Если движение не началось , ждем
            if not movement_started:
                if time.time() - last_feedback_time > 2:
                    print("Waiting for movement to start...")
                    last_feedback_time = time.time()
            time.sleep(0.1)
        
        print(f"Timeout waiting for movement completion after {timeout} seconds")
        return False
    
    def send_command(self, command):
        """Отправка команды манипулятору"""
        # Сбрасываем флаги перед отправкой новой команды
        with self.lock:
            self.feedback_received = False
            self.is_moving = False
        self.udp_socket.sendto(command.encode(), self.target_address)
    
    def send_g_command(self, X, Y, T, Z, V):
        """Формирует команды в формате g:x:y:t:z:v# с ожиданием завершения"""
        command = f"g:{X}:{Y}:{T}:{Z}:{V}#"
        print(f"Sending: {command}")
        self.send_command(command)
        
        # Ждем завершения движения перед следующим действием
        if not self.wait_for_completion():
            print("Warning: Movement may not have completed properly")
    
    def moving(self):
        """Выполнение последовательности команд с ожиданием обратной связи"""
        print("Start of manipulator movement")
        # Время для получения первой обратной связи
        time.sleep(1)
            
        self.send_g_command(200, 100, 0, 60, 0)
        self.send_g_command(200, 100, 0, 40, 0)
        self.send_g_command(200, 100, 0, 35, 1)
        
        self.send_g_command(200, 100, 0, 150, 1)
        self.send_g_command(100, 150, 0, 150, 1)
        self.send_g_command(100, 150, 0, 90, 1)
        self.send_g_command(100, 150, 0, 45, 1)
        
        self.send_g_command(100, 150, 0, 45, 0)
        
        self.send_g_command(100, 150, 0, 70, 0)
        
        self.send_g_command(20, 200, 0, 150, 0)
        self.send_g_command(20, 200, 0, 60, 0)
        self.send_g_command(20, 200, 0, 33, 1)
        
        self.send_g_command(20, 200, 0, 75, 1)
        
        self.send_g_command(20, 200, 0, 150, 1)
        self.send_g_command(100, 150, 0, 150, 1)
        self.send_g_command(100, 150, 0, 120, 1)
        self.send_g_command(100, 150, 0, 90, 1)
        
        self.send_g_command(100, 150, 0, 65, 0)
        
        self.send_g_command(100, 150, 0, 150, 0)
    
        print("The manipulator movement is complete")
    
    def get_feedback_buffer(self):
        """Получение последних сообщений обратной связи"""
        return list(self.feedback_buffer)
    
    def close(self):
        """Закрытие сокета"""
        self.listening = False
        time.sleep(0.5)  # Даем время потоку завершиться
        self.udp_socket.close()
        print("Socket closed")

# основная программа
if __name__ == "__main__":
    # Создание экземпляра контроллера
    manipulator = ManipulatorController("192.168.42.241", 8888)
    
    try:
        # Выполнение последовательности один раз
        manipulator.moving()
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Закрытие соединения
        manipulator.close()