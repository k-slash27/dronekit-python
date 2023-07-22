from dronekit import connect, VehicleMode, Command
import time, linecache
from pymavlink import mavutil

# ウェイポイント
waypoints = {
    1: {'lat': 35.760215, 'long': 140.379330, 'alt': 100},
    2: {'lat': 35.878275, 'long': 140.338069, 'alt': 100},
    3: {'lat': 35.878275, 'long': 140.338069, 'alt': 100}, #dummy
}

# 接続
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=120)

cmds = vehicle.commands

initial_alt=100

# アームチェック
@vehicle.on_attribute('armed')   
def decorated_armed_callback(self, attr_name, value):
    print(" Armed:", value)


# モード変更チェック
@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    print(" Mode changed to", value)

# フライトプラン書き込み
def adds_mission(waypoints):
    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
     
    # 離陸できていなければ離陸
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, initial_alt))

    # ウェイポイント書き込み
    for wp in waypoints.values():
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp.get('lat'), wp.get('long'), wp.get('alt')))
    print(" Upload new commands to vehicle")
    cmds.upload()

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
print('LAND')
vehicle.mode = VehicleMode("QLAND")

# vehicleオブジェクトの終了
print("Close vehicle object")
vehicle.close()
