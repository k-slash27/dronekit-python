from dronekit import connect, VehicleMode, Command
import time, linecache
from pymavlink import mavutil

# ウェイポイント
# waypoints = {
#     1: {'lat': 35.8791612185214817, 'long': 140.339347422122955, 'alt': 10},
#     2: {'lat': 35.8789525840509356, 'long': 140.339042991399765, 'alt': 10},
#     3: {'lat': 35.8790492948894482, 'long': 140.339427888393402, 'alt': 10},
#     4: {'lat': 35.8788221872277973, 'long': 140.339182466268539, 'alt': 10},
#     5: {'lat': 35.8789417176572059, 'long': 140.339532494544983, 'alt': 10},
#     6: {'lat': 35.8787146096871084, 'long': 140.339320600032806, 'alt': 10},
#     7: {'lat': 35.8788526131723913, 'long': 140.339646488428116, 'alt': 10},
#     8: {'lat': 35.8791612185214817, 'long': 140.339347422122955, 'alt': 10},
#     9: {'lat': 35.8791612185214817, 'long': 140.339347422122955, 'alt': 10}, #dummy
# }
waypoints = 'mp.txt'

# 接続
# vehicle = connect('192.168.2.3:14550', wait_ready=True, timeout=120)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=120)

cmds = vehicle.commands

initial_alt=0

# アームチェック
@vehicle.on_attribute('armed')   
def decorated_armed_callback(self, attr_name, value):
    print(" Armed:", value)


# モード変更チェック
@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    print(" Mode changed to", value)


# フライトプランファイル読み込み
def readmission(aFileName):
    print("\nReading mission from file: %s" % aFileName)
    
    with open(aFileName) as f:
        missionlist=[]
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            elif i==1:
                continue
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
        return missionlist


# フライトプラン書き込み
def adds_mission(waypoints):
    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
     
    # 離陸できていなければ離陸
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, initial_alt))

    # フライトプランファイル読み込み
    missionlist = readmission(waypoints)

    # ウェイポイント書き込み
    # for wp in waypoints.values():
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp.get('lat'), wp.get('long'), wp.get('alt')))
    for command in missionlist:
        cmds.add(command)

    print(" Upload new commands to vehicle")
    cmds.upload()


# 高度初期値をファイルから取得
def get_inital_alt(aFileName):
    line = linecache.getline(aFileName, 3).rstrip("\n")
    print(line)
    linearray=line.split('\t')
    alt = float(linearray[10])
    return int(alt)


# アーム・離陸
def arm_and_takeoff(aTargetAltitude):
    # ホームロケーション取得
    print("Set home location")
    while not vehicle.home_location:
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print(" Waiting for home location ...")
    #ホームロケーションの取得完了
    print(" Home location: %s " % vehicle.home_location)
    # アーム
    print("Pre-arm check")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print(" Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # 離陸
    print("Taking off!")
    print(aTargetAltitude)
    vehicle.simple_takeoff(aTargetAltitude) 
    # 安全な高度に達するまで待つ
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)



# FCにフライトプランを書き込み
print('Create a new mission')
adds_mission(waypoints)

# アームして離陸
initial_alt = get_inital_alt(waypoints)
arm_and_takeoff(initial_alt)

# 次のポジションの初期化
vehicle.commands.next = 0

print("Starting mission")

# AUTOモードに変更しmissionスタート
vehicle.mode = VehicleMode("AUTO")

while True:
    nextwaypoint = vehicle.commands.next
    # 次の移動先が最終地点の場合、mission終了
    if nextwaypoint >= vehicle.commands.count:
        print("End of mission")
        break

    print(' Next waypoint: %s' % (nextwaypoint))  
    time.sleep(1)


# RTLモードに切り替えてホームロケーションに戻る
print('Return to launch')
vehicle.mode = VehicleMode("RTL")

# vehicleオブジェクトの終了
print("Close vehicle object")
vehicle.close()
