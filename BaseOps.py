




def GetBlocks_Base(DesiredSig):
    
    class Blocks (Structure):
        fields_ = [ ("type", c_uint),
                   ("signature", c_uint),
                   ("x", c_uint),
                   ("y", c_uint),
                   ("width", c_uint),
                   ("height", c_uint),
                   ("angle", c_uint) ]
    
    blocks = BlockArray(3)
    frame  = 0
    
    # Wait for blocks #
    
    while 1:
        count = pixy_get_blocks(3, blocks)
        if count > 0:
            # Blocks found #
            #            print 'frame %3d:' % (frame)
            frame = frame + 1
            for index in range (0, count):
                if int(blocks[index].signature)=DesiredSig:
                    x=int(blocks[index].x)
                    y=int(blocks[index].y)
                    w=int(blocks[index].width)

                    time.sleep(1)   ## **change when perfected
                    return x,y,w;
                else:
                    index=index +1

################################ Centering Function ################################

def Centering_Base(DesiredSig):
    print("Centering Function:")
    duration=.5
    while True:


        x,y,w= GetBlocks_Base(DesiredSig)
                
        if x< 145:        #sets Xaxis based on Pixy coordinate
            Xaxis= 1;
            print('move west')
        
        
        elif x>175:
            Xaxis=-1;
            print('move east')
        
        else:
            Xaxis=0;
            print('X is centered')
        
        
        if y< 105:        #sets Yaxis based on Pixy coordinate
            Yaxis=-1;
            print('move north')
        
        elif y>135:
            Yaxis=1;
            print('move south')
  
        else:
            Yaxis=0;
            print('Y is centered')

        if w < 145:        #sets Zaxis based on Pixy coordinate
            Zaxis= 1;
            print('decend')
        
        
        else:
            Zaxis=0;
            print('W is centered')

        send_ned_velocity(Xaxis,Yaxis,Zaxis,duration) #move uas to desired location

        print('*************************')
        
        if Xaxis==0 and Yaxis==0 and Zaxis==0:        #Exits if within hit box
            print('all centered...SUCK IT KEVIN')
            break;
         

    time.sleep(1)
################################ Velocity ################################

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
        Move vehicle in direction based on specified velocity vectors.
        """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
           0,       # time_boot_ms (not used)
           0, 0,    # target system, target component
           mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
           0b0000111111000111, # type_mask (only speeds enabled)
           0, 0, 0, # x, y, z positions (not used)
           velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
           0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
           0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink))
        
           # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
    vehicle.send_mavlink(msg)
    time.sleep(1)


################################ BaseOps ################################

    def BaseOps:
      DesiredSig=4                #indicates Base signature
      Centering_Base(DesiredSig)  #lower and center onto base
      vehicle.armed=False         #disarm motors while landed  
      print "landed and disarmed"
