import pygame
from socket import *


FILL_COLOR = (255, 255, 255)    # white


def setting():
    sock = socket(AF_INET, SOCK_STREAM)
    sock.connect(('192.168.1.4', 1972))

    pygame.init()
    screen = pygame.display.set_mode((500, 500))
    clock = pygame.time.Clock()
    return sock, screen, clock


def main(joy_stick):
    sock, screen, clock = setting()

    # 조이 스틱을 사용할 때
    if joy_stick:
        pygame.joystick.init()

        joystick = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())][0]
        screen.fill(FILL_COLOR)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sock.send("exit".encode('utf-8'))
                    pygame.quit()
                    exit()

            degree = str(int(joystick.get_axis(0) * 80))
            speed = str(int(joystick.get_axis(2) * (-50)))

            while len(degree) != 3:
                if len(degree) < 3:
                    degree = '0' + degree
                else:
                    break

            while len(speed) != 3:
                if len(speed) < 3:
                    speed = '0' + speed
                else:
                    break

            sock.send((degree + speed).encode('utf-8'))
            while sock.recv(1024).decode('utf-8') != "ok":
                pass

            pygame.display.update()
            clock.tick(20)

    # 조이스틱을 사용하지 않을 때(키보드 사용)
    else:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sock.send("exit".encode('utf-8'))
                    pygame.quit()
                    exit()

                screen.fill(FILL_COLOR)

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        sock.send('up'.encode('utf-8'))
                    elif event.key == pygame.K_DOWN:
                        sock.send('down'.encode('utf-8'))
                    elif event.key == pygame.K_LEFT:
                        sock.send('left'.encode('utf-8'))
                    elif event.key == pygame.K_RIGHT:
                        sock.send('right'.encode('utf-8'))
                    elif event.key == pygame.K_1:
                        sock.send('one'.encode('utf-8'))
                    elif event.key == pygame.K_2:
                        sock.send('two'.encode('utf-8'))
                    elif event.key == pygame.K_3:
                        sock.send('thr'.encode('utf-8'))
                    elif event.key == pygame.K_4:
                        sock.send('for'.encode('utf-8'))
                    elif event.key == pygame.K_5:
                        sock.send('fiv'.encode('utf-8'))

                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                        sock.send('keyupbldc'.encode('utf-8'))
                    elif event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                        sock.send('keyupservo'.encode('utf-8'))

            pygame.display.update()
            clock.tick(20)


if __name__ == "__main__":
    main(True)
