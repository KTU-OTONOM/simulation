#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smart_can_msgs.msg import Rcunittoomux, Rcthrtdata, Autonomousbrakepedalcontrol, Autonomoushbmotorcontrol, Autonomoussteeringmotcontrol
import time
import sys, select

import tty,termios

class RobotaxiSurucu(Node):
    def __init__(self):
        super().__init__('robotaksi_surucu')
        
        self.steering_pub = self.create_publisher(Autonomoussteeringmotcontrol, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.unittomux_pub = self.create_publisher(Rcunittoomux, '/beemobs/rc_unittoOmux', 10)
        self.thrt_pub = self.create_publisher(Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.brake_pub = self.create_publisher(Autonomousbrakepedalcontrol, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.hb_pub = self.create_publisher(Autonomoushbmotorcontrol, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.msg_unittomux = Rcunittoomux()
        self.msg = Autonomousbrakepedalcontrol()
        self.msg_ = Rcthrtdata()
        
        self.msg_.rc_thrt_pedal_position=50

        print("Robotaksi Surucu")
        
        while True:
            self.settings = termios.tcgetattr(sys.stdin)
            key=self.getKey()
            
            if key=="i":
                self.msg_unittomux.autonomous_emergency = 0
                self.msg_unittomux.rc_windowresintance = 0
                self.msg_unittomux.rc_interiorlight = 0
                self.msg_unittomux.rc_reverselight = 0
                self.msg_unittomux.rc_drl = 0
                self.msg_unittomux.rc_brakelight = 0
                self.msg_unittomux.autonomous_door_open = 0
                self.msg_unittomux.autonomous_door_close = 0
                self.msg_unittomux.rc_signalstatus = 0
                self.msg_unittomux.rc_lowbeam = 0
                self.msg_unittomux.rc_highbeam = 0
                self.msg_unittomux.rc_selectiongear = 0
                self.msg_unittomux.rc_ignition = 1
                self.unittomux_pub.publish(self.msg_unittomux)
                self.get_logger().info("beemobs start")
                
                msghb = Autonomoushbmotorcontrol()
                msghb.autonomous_hb_motor_pwm = 200
                msghb.autonomous_hb_motstate = 0
                msghb.autonomous_hb_moten = 1
                self.hb_pub.publish(msghb)
            if key=="o":
                msg_ = Rcthrtdata()
                msg_.rc_thrt_pedal_position = 0
                msg_.rc_thrt_pedal_press = 1
                self.thrt_pub.publish(msg_)
                msg = Autonomousbrakepedalcontrol()
                msg.autonomous_brakemotor_voltage = 0
                msg.autonomous_brakepedalmotor_per = 0
                msg.autonomous_brakepedalmotor_acc = 0
                msg.autonomous_brakepedalmotor_en = 0
                self.brake_pub.publish(msg)
                
                self.msg_unittomux = Rcunittoomux()
                self.msg_unittomux.autonomous_emergency = 0
                self.msg_unittomux.rc_windowresintance = 0
                self.msg_unittomux.rc_interiorlight = 0
                self.msg_unittomux.rc_reverselight = 0
                self.msg_unittomux.rc_drl = 0
                self.msg_unittomux.rc_brakelight = 0
                self.msg_unittomux.autonomous_door_open = 0
                self.msg_unittomux.autonomous_door_close = 0
                self.msg_unittomux.rc_signalstatus = 0
                self.msg_unittomux.rc_lowbeam = 0
                self.msg_unittomux.rc_highbeam = 0
                self.msg_unittomux.rc_selectiongear = 0
                self.msg_unittomux.rc_ignition = 0
                self.unittomux_pub.publish(self.msg_unittomux)
                self.get_logger().info("beemobs stop")
            if key=='d':
                steering_msg = Autonomoussteeringmotcontrol()
                steering_msg.autonomous_steeringmot_en = 1
                steering_msg.autonomous_steeringmot_pwm = 200
                self.steering_pub.publish(steering_msg)
                
            elif key=='a':
                steering_msg = Autonomoussteeringmotcontrol()
                steering_msg.autonomous_steeringmot_en = 1
                steering_msg.autonomous_steeringmot_pwm = 70
                self.steering_pub.publish(steering_msg)
            
            else:
                steering_msg = Autonomoussteeringmotcontrol()
                steering_msg.autonomous_steeringmot_en = 0
                steering_msg.autonomous_steeringmot_pwm = 0
                self.steering_pub.publish(steering_msg)
                
                
            if key=='j':
                self.msg_unittomux.rc_selectiongear = 1
                self.unittomux_pub.publish(self.msg_unittomux)
                self.get_logger().info("beemobs vites ileri")
            if key=='m':
                self.msg_unittomux.rc_selectiongear = 2
                self.unittomux_pub.publish(self.msg_unittomux)
                self.get_logger().info("beemobs vites geri")
                
            if key=='x':
                self.msg_unittomux.rc_selectiongear = 0
                self.unittomux_pub.publish(self.msg_unittomux)


                self.msg.autonomous_brakemotor_voltage = 1
                self.msg.autonomous_brakepedalmotor_per +=10
                self.msg.autonomous_brakepedalmotor_acc = 10000
                self.msg.autonomous_brakepedalmotor_en = 1
                
                if self.msg.autonomous_brakepedalmotor_per>100:
                    self.msg.autonomous_brakepedalmotor_per=100
                
                self.brake_pub.publish(self.msg)
                
            if key=='s':
                self.msg_unittomux.rc_selectiongear = 0
                self.unittomux_pub.publish(self.msg_unittomux)
                
                
                self.msg_.rc_thrt_pedal_position =45


                self.msg.autonomous_brakemotor_voltage = 1
                self.msg.autonomous_brakepedalmotor_per =0
                self.msg.autonomous_brakepedalmotor_acc = 10000
                self.msg.autonomous_brakepedalmotor_en = 1
                
                self.brake_pub.publish(self.msg)
                
            if key=='w':
                
                self.msg_.rc_thrt_pedal_position += 2
                self.msg_.rc_thrt_pedal_press = 0
                
                self.get_logger().info(f"{self.msg_.rc_thrt_pedal_position}")
                
            if key=='z':
                
                self.msg_.rc_thrt_pedal_position -= 5
                self.msg_.rc_thrt_pedal_press = 0
                
                self.get_logger().info(f"{self.msg_.rc_thrt_pedal_position}")
                
            if key=='n':
                msghb = Autonomoushbmotorcontrol()
                msghb.autonomous_hb_motor_pwm = 200
                msghb.autonomous_hb_motstate = 1
                msghb.autonomous_hb_moten = 1
                self.hb_pub.publish(msghb)
                self.get_logger().info("el freni indir ")
                
            if key=='b':
                msghb = Autonomoushbmotorcontrol()
                msghb.autonomous_hb_motor_pwm = 200
                msghb.autonomous_hb_motstate = 0
                msghb.autonomous_hb_moten = 1
                self.hb_pub.publish(msghb)
                self.get_logger().info("el freni indir ")
                
            if self.msg_.rc_thrt_pedal_position>255:
                self.msg_.rc_thrt_pedal_position=255
            
            self.thrt_pub.publish(self.msg_)
           
            if key=="\x03":
                    break


        print('''                
            try:
                events = inputs.get_gamepad()

                for event in events:
                    self.get_logger().info(f'{event.ev_type}, {event.code}, {event.state}')
                    if event.code == 'ABS_HAT0Y' and event.state == -1:
                        self.msg_unittomux.autonomous_emergency = 0
                        self.msg_unittomux.rc_windowresintance = 0
                        self.msg_unittomux.rc_interiorlight = 0
                        self.msg_unittomux.rc_reverselight = 0
                        self.msg_unittomux.rc_drl = 0
                        self.msg_unittomux.rc_brakelight = 0
                        self.msg_unittomux.autonomous_door_open = 0
                        self.msg_unittomux.autonomous_door_close = 0
                        self.msg_unittomux.rc_signalstatus = 0
                        self.msg_unittomux.rc_lowbeam = 0
                        self.msg_unittomux.rc_highbeam = 0
                        self.msg_unittomux.rc_selectiongear = 1
                        self.msg_unittomux.rc_ignition = 1

                    if event.code == 'ABS_HAT0Y' and event.state == 1:
                        can_id = 0x0000800
                        can_data = [3, 0, 0, 0, 0, 0, 0, 0]
                        self.send_can_message(can_id, can_data)
                        self.get_logger().info("beemobs thrt geri")

                    if event.code == 'BTN_START' and event.state:
                        self.msg_unittomux.autonomous_emergency = 0
                        self.msg_unittomux.rc_windowresintance = 0
                        self.msg_unittomux.rc_interiorlight = 0
                        self.msg_unittomux.rc_reverselight = 0
                        self.msg_unittomux.rc_drl = 0
                        self.msg_unittomux.rc_brakelight = 0
                        self.msg_unittomux.autonomous_door_open = 0
                        self.msg_unittomux.autonomous_door_close = 0
                        self.msg_unittomux.rc_signalstatus = 0
                        self.msg_unittomux.rc_lowbeam = 0
                        self.msg_unittomux.rc_highbeam = 0
                        self.msg_unittomux.rc_selectiongear = 0
                        self.msg_unittomux.rc_ignition = 1
                        self.unittomux_pub.publish(self.msg_unittomux)
                        self.get_logger().info("beemobs start")
                        #fren
                        # msg = Autonomousbrakepedalcontrol()

                        # msg.AUTONOMOUS_BrakeMotor_Voltage = 1
                        # msg.AUTONOMOUS_BrakePedalMotor_PER = 0
                        # msg.AUTONOMOUS_BrakePedalMotor_ACC = 0
                        # msg.AUTONOMOUS_BrakePedalMotor_EN = 1

                        # self.brake_pub.publish(msg)
                        #el freni
                        msghb = Autonomoushbmotorcontrol()

                        msghb.autonomous_hb_motor_pwm = 200
                        msghb.autonomous_hb_motstate = 0
                        msghb.autonomous_hb_moten = 1

                        self.hb_pub.publish(msghb)

                    if(event.code == 'BTN_SELECT' and event.state):
                        
                        msg_ = Rcthrtdata()
                        msg_.rc_thrt_pedal_position = 0
                        msg_.rc_thrt_pedal_press = 1
                        self.thrt_pub.publish(msg_)

                        msg = Autonomousbrakepedalcontrol()

                        msg.autonomous_brakemotor_voltage = 0
                        msg.autonomous_brakepedalmotor_per = 0
                        msg.autonomous_brakepedalmotor_acc = 0
                        msg.autonomous_brakepedalmotor_en = 0

                        self.brake_pub.publish(msg)

                        #el freni
                        # msghb = AUTONOMOUS_HB_MotorControl()

                        # msghb.AUTONOMOUS_HB_Motor_PWM = 200
                        # msghb.AUTONOMOUS_HB_MotState = 2
                        # msghb.AUTONOMOUS_HB_MotEN = 1

                        # self.hb_pub.publish(msghb)
                        # rospy.sleep(0.5)
                        # msghb.AUTONOMOUS_HB_MotEN = 0

                        # self.hb_pub.publish(msghb)

                        self.msg_unittomux = Rcunittoomux()
                        self.msg_unittomux.autonomous_emergency = 0
                        self.msg_unittomux.rc_windowresintance = 0
                        self.msg_unittomux.rc_interiorlight = 0
                        self.msg_unittomux.rc_reverselight = 0
                        self.msg_unittomux.rc_drl = 0
                        self.msg_unittomux.rc_brakelight = 0
                        self.msg_unittomux.autonomous_door_open = 0
                        self.msg_unittomux.autonomous_door_close = 0
                        self.msg_unittomux.rc_signalstatus = 0
                        self.msg_unittomux.rc_lowbeam = 0
                        self.msg_unittomux.rc_highbeam = 0
                        self.msg_unittomux.rc_selectiongear = 0
                        self.msg_unittomux.rc_ignition = 0

                        self.unittomux_pub.publish(self.msg_unittomux)
                        self.get_logger().info("beemobs stop")
                        
                    if(event.code == 'BTN_SOUTH' and event.state):
                        self.msg_unittomux.rc_selectiongear = 2
                        self.unittomux_pub.publish(self.msg_unittomux)

                        self.get_logger().info("beemobs vites geri")

                    if(event.code == 'BTN_NORTH' and event.state ==1):
                        msghb = Autonomoushbmotorcontrol()

                        msghb.autonomous_hb_motor_pwm = 200
                        msghb.autonomous_hb_motstate = 0
                        msghb.autonomous_hb_moten = 1

                        self.hb_pub.publish(msghb)

                        self.get_logger().info("el freni cek ")

                    if(event.code == 'BTN_WEST' and event.state):
                        self.msg_unittomux.rc_selectiongear = 1
                        self.unittomux_pub.publish(self.msg_unittomux)

                        self.get_logger().info("beemobs vites ileri")
                    
                    if(event.code == 'BTN_EAST' and event.state):
                        msghb = Autonomoushbmotorcontrol()

                        msghb.autonomous_hb_motor_pwm = 200
                        msghb.autonomous_hb_motstate = 1
                        msghb.autonomous_hb_moten = 1

                        self.hb_pub.publish(msghb)

                        self.get_logger().info("el freni indir ")

                    if event.code == 'ABS_X':
                        if event.state > 7500: #sag
                            self.get_logger().info("sag")
                            steering_msg = Autonomoussteeringmotcontrol()
                            steering_msg.autonomous_steeringmot_en = 1
                            steering_msg.autonomous_steeringmot_pwm = 200
                            self.steering_pub.publish(steering_msg)
                        elif event.state < -7500: # sol
                            steering_msg = Autonomoussteeringmotcontrol()
                            steering_msg.autonomous_steeringmot_en = 1
                            steering_msg.autonomous_steeringmot_pwm = 70
                            self.steering_pub.publish(steering_msg)
                        else :
                            steering_msg = Autonomoussteeringmotcontrol()
                            steering_msg.autonomous_steeringmot_en = 0
                            steering_msg.autonomous_steeringmot_pwm = 0
                            self.steering_pub.publish(steering_msg)

                    if event.code == 'ABS_RY':
                        
                        if event.state > 7000: #geri

                            self.msg_unittomux.rc_selectiongear = 0
                            self.unittomux_pub.publish(self.msg_unittomux)

                            msg = Autonomousbrakepedalcontrol()

                            msg.autonomous_brakemotor_voltage = 1
                            msg.autonomous_brakepedalmotor_per = 0
                            msg.autonomous_brakepedalmotor_acc = 10000
                            msg.autonomous_brakepedalmotor_en = 1
                            if event.state > 7100:
                                z = (100 - 30) * (event.state - 7100) / (34000 - 7100) + 30
                                msg.autonomous_brakepedalmotor_per = int(z)

                            self.brake_pub.publish(msg)
                            
                            self.get_logger().info("beemobs frene basildi")
                        if event.state < -400: # ileri

                            # msg = AUTONOMOUS_BrakePedalControl()

                            # msg.AUTONOMOUS_BrakeMotor_Voltage = 1
                            # msg.AUTONOMOUS_BrakePedalMotor_PER = 0
                            # msg.AUTONOMOUS_BrakePedalMotor_ACC = 0
                            # msg.AUTONOMOUS_BrakePedalMotor_EN = 1

                            # self.brake_pub.publish(msg)

                            msg_ = Rcthrtdata()
                            msg_.rc_thrt_pedal_position = 0
                            if event.state < -7500:
                                y = (120 - 50) * (event.state - -7500) / (-33000 - -7500) + 50
                                msg_.rc_thrt_pedal_position = int(y)

                            msg_.rc_thrt_pedal_press = 0
                            self.thrt_pub.publish(msg_)
            except Exception as e:
                self.get_logger().error(f'Exception in monitor_gamepad: {e}')
                continue
        
''')
    def getKey(self):

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
        
        #self.timer=self.create_timer(1.0,self.geri_git)
        
        self.get_logger().info("a")
        time.sleep(1)
        self.get_logger().info("b")
        

    def start(self):
        self.ignition_msg.rc_ignition = 1  # Aracı çalıştır
        self.ignition_publisher.publish(self.ignition_msg)
        self.get_logger().info(f"{self.ignition_msg}")
        self.get_logger().info("Arac çaliştirildi.")
        self.throttle_msg.rc_thrt_pedal_press = 1
        self.throttle_publisher.publish(self.throttle_msg)
        

    def stop(self):
        self.ignition_msg.rc_ignition = 0  # Aracı durdur
        self.ignition_msg.rc_reverselight=0
        self.ignition_publisher.publish(self.ignition_msg)
        self.get_logger().info("Arac durduruldu.")

    def ileri_git(self, hiz):
        self.ignition_msg.rc_selectiongear = 1
        self.throttle_msg.rc_thrt_pedal_press = 1  # Gaza bas
        self.throttle_msg.rc_thrt_pedal_position = hiz
        self.ignition_publisher.publish(self.ignition_msg)
        self.throttle_publisher.publish(self.throttle_msg)
        self.get_logger().info(f"Arac {hiz} hiziyla ileri gidiyor.")

    def geri_git(self):
        self.throttle_publisher.publish(self.throttle_msg)
        self.get_logger().info("e")

    def fren(self):
        self.brake_msg.autonomous_brakepedalmotor_per = 100
        self.throttle_msg._rc_thrt_pedal_press=1
        self.brake_publisher.publish(self.brake_msg)
        self.get_logger().info("f")

    def sinyalleri_ac(self, yon):
        if yon == "sol":
            self.signal_msg.rc_signalstatus = 2
        elif yon == "sag":
            self.signal_msg.rc_signalstatus = 1
        else:  # varsayılan olarak dörtlüleri yak
            self.signal_msg.rc_signalstatus = 3
        self.signal_publisher.publish(self.signal_msg)
        self.get_logger().info(f"{yon} sinyaller acik.")

    def farlari_ac(self):
        self.signal_msg.rc_lowbeam = 1
        self.signal_msg.rc_highbeam = 1
        self.signal_publisher.publish(self.signal_msg)
        self.get_logger().info("Farlar acik.")
        
    def el_frenini_indir(self):
        self.handbrake_msg.autonomous_hb_moten=1
        self.handbrake_msg.autonomous_hb_motstate=1
        self.handbrake_msg._autonomous_hb_motor_pwm=200
        self.handbrake_publisher.publish(self.handbrake_msg)
        

def main(args=None):
    rclpy.init(args=args)
    surucu = RobotaxiSurucu()
    rclpy.spin(surucu)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
