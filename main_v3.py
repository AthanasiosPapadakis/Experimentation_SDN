import glob
import os
import sys
import argparse
import logging
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

#create data SN / XUPPER / XLOWER / YUPPER / YLOWER
data = [["1", 410, 330, 70, -10], 
        ["2", 410, 330, 140, 70], 
        ["3", 410, 330, 210, 140],
        ["4", 410, 330, 280, 210], 
        ["5", 410, 330, 340, 280], 
        ["6", 330, 260, 70, -10],
        ["7", 330, 260, 210, 120], 
        ["8", 330, 260, 340, 280], 
        ["9", 260, 190, 70, -10],
        ["10", 260, 190, 210, 120], 
        ["11", 260, 190, 340, 280], 
        ["12", 190, 120, 70, -10],
        ["13", 190, 120, 210, 120], 
        ["14", 190, 120, 340, 280], 
        ["15", 120, 50, 70, -10],
        ["16", 120, 50, 140, 70], 
        ["17", 120, 50, 210, 140], 
        ["18", 120, 50, 280, 210],
        ["19", 120, 50, 340, 280], 
        ["20", 50, -10, 70, -10], 
        ["21", 50, -10, 140, 70],        
        ["22", 50, -10, 210, 140],
        ["23", 50, -10, 280, 210],
        ["24", 50, -10, 340, 280]]
  
#define header names
col_names = ["StaticNode", "X UpperBound", "X LowerBound", "Y UpperBound", "Y LowerBound"]

def main():

    #Open file to write
    f1 = open('mobility100_1.txt', 'w')
    f2 = open('raw100_1.txt', 'w')
    
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:

        world = client.get_world()
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None
        
        # Start recording        
        # Highlight the r before the path, it translates the path as raw allowing the proper identification of the path
        client.start_recorder(r"C:/Users/panoramix/AppData/Local/CarlaUE4/Saved/demo01.log")
        
        #Create an array with the vehicles
        vehicles = []
                
        #Count for how many vehicles want to spawn
        for count in range (0, 100) :
        
            # Spawn ego vehicle
            ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
            ego_bp.set_attribute('role_name','ego')
            ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
            ego_bp.set_attribute('color',ego_color)
            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)
            
            #List the spawn points
            spawn_point_list = world.get_map().get_spawn_points()
            
            if 0 < number_of_spawn_points:
                
                random.shuffle(spawn_points)
                ego_transform = spawn_points[0]
                
                try:
                    ego_vehicle = world.spawn_actor(ego_bp,ego_transform)    
                #RuntimeError: Spawn failed because of collision at spawn position                    
                except RuntimeError:
                    #print("Collision, respawning")
                    pass
                
                # Add GNSS sensor to ego vehicle. 
                gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
                gnss_location = carla.Location(0,0,0)
                gnss_rotation = carla.Rotation(0,0,0)
                gnss_transform = carla.Transform(gnss_location,gnss_rotation)
                #Set the time you are going to retrieve data
                gnss_bp.set_attribute("sensor_tick",str(1.0))
                ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
                
                # Enable autopilot for ego vehicle
                ego_vehicle.set_autopilot(True)
                
                #Fill the array with the created vehicle
                vehicles.append(ego_vehicle)
                
            else: 
                logging.warning('Could not found any spawn points')
        
        def gnss_callback(gnss):
            for i,v in enumerate(vehicles) :
                location = v.get_location()
                i = i + 1
                #a is the number of STATIC NODES
                for a in range(24):
                    if( location.x <= data[a][1] and location.x >= data[a][2] and location.y <= data[a][3] and location.y >= data[a][4]):
                        #This is to understand what is the output, not really efficient for data analysis
                        #print('Vehicle:%s ' % i + 'at t=%s ' % int(gnss.timestamp) + 'is at (x,y):(%s,' % int(location.x) + '%s) ' % int(location.y) + 'within %s' % data[a][0])
                        #This format is efficient for data analysis
                        #print('%s' % i + ' %s' % int(gnss.timestamp) + ' %s' % data[a][0])
                        print('%s' % (i+24) + ' %s' % int(gnss.timestamp) + ' %s' % int(location.x) + ' %s' % int(location.y), file=f1)
                        print('%s' % (i+24) + ' %s' % int(gnss.timestamp) + ' %s' % data[a][0], file=f2)
                        break
        ego_gnss.listen(lambda gnss: gnss_callback(gnss))

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        while True:
            world_snapshot = world.wait_for_tick()

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')