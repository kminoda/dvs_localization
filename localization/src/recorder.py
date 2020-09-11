import rosbag
import rospy
import roslaunch
import time
import argparse

WORKDIR_PATH = '/home/koji/git/dvs_localziation/'

def record(filename, recording_time):
    launch_path = WORKDIR_PATH+'localization/src/record.launch'

    with open(launch_path, 'w') as f:
        f.write('<launch>\n')
        f.write('    <node pkg="rosbag" type="record" name="rosbag_record" args="record -a -O {}"/>\n'.format(filename))
        f.write('</launch>\n')

    launch_path = WORKDIR_PATH+'localization/src/record.launch'
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments([launch_path])[0])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    launch.start()
    time.sleep(recording_time)
    launch.shutdown()
    print('Finished recording')

if __name__=='__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('--filename', help='rosbag', type=str, default=WORKDIR_PATH+'localization/test.bag')
    parser.add_argument('--time', help='time', type=float, default=1.0)
    args = parser.parse_args()

    record(args.filename, args.time)