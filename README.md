# teleop

# Ogni myo ha un nome in base alla sua destinazione (esempio For_r), quindi inserirli in modo corretto.
# I topic in cui pubblicheranno saranno preceduti dal proprio nome.
# Avviare i myo singolarmente (mandarli in un unico launch non funziona).
- rosrun ros_myo myo-rawNode.py /dev/ttyACM0 -c con0
- rosrun ros_myo myo-rawNode.py /dev/ttyACM1 -c con1
- rosrun ros_myo myo-rawNode.py /dev/ttyACM2 -c con2
- rosrun ros_myo myo-rawNode.py /dev/ttyACM3 -c con3

# Se le imu qb e le mani del kuka girano sullo stesso pc inserire prima la usb delle imu (poichè richiedono usb0). Mandare il launch della qb interface:
- roslaunch qb_interface qb_interface.launch

# Mandare la kinect su Matlab

# Mandare il launch per calcolare l'orientazione dei vari link
- roslaunch ros_myo extended_madgw.launch 

# Madare il launch per la ricostruzione della postura delle braccia e i riferimenti per stiffness/chiusura soft_hand 
- roslaunch ros_myo motion_cap.launch

Tutti i paramentri sono configurabili in select_link.yaml presente nella cartella config

