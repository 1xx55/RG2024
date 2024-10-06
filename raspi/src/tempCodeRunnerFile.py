
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x01\x0E'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")


                    elif average_x > x_high + mergin:  # move 90
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\x5A'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")

                    elif average_x < x_low:
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x01\x0E'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        x_done=1
                    elif average_x > x_high:
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\x5A'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        x_done=1
                    else:
                        command = b'\x66\x66\x07'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        x_done = 1
                        flag+=1

                if (y_done == 0 and x_done==1) or flag>0:
                    if average_y < y_low - mergin:  # move 180
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\xB4'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")

                    elif average_y > y_high + mergin:  # move 0
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\x00'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")


                    elif average_y < y_low:  # move 180
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\xB4'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        y_done=1
                    elif average_y > y_high:  # move 0
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        command = b'\x00\x00'
                        env.ser.write(command)
                        #print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        #print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        y_done=1