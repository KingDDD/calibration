#!/usr/bin/python
import argparse
import json
import os
import glob
import numpy as np
import yaml
import logging
from pyquaternion import Quaternion
from shutil import copyfile

logger = logging.getLogger('logger')
logger.setLevel(logging.DEBUG)

# sync delay from Vicon sync out to headphone jack
SYNC_DELAY = 2995000

# exposure time of Vicon: 1ms
VICON_EXPOSURE_TIME = 1e6

# vicon fps
VICON_FPS = 120

# sync delay in Vicon frame count
SYNC_DELAY_IN_FRAME = (SYNC_DELAY + VICON_EXPOSURE_TIME / 2.0) / 1e9 * VICON_FPS

# mse tolerance
MSE_TOLERANCE = 0.09

# maximum number of frames dropped
FRAME_DROP_TOLERANCE = 200

# frame_id range for timestamp extrapolate
FRAME_EXTRAPOLATE_LIMIT = 10 * VICON_FPS

# timestamp range for frame_id extrapolate
TIMESTAMP_EXTRAPOLATE_LIMIT = 10 * 1e9

# get a dictionary from wearable time to vicon frame id
# input: decoded sync event file
#  { "ktime": 1194989711, "mtime": 1209759826, "payload_present": true,
#    "frame_number": 2045758, "mean_squared_error": 0.000607824 }
# output: dictionary from wearable time to vicon frame id
def process_sync(file_path):
    dict = {}
    reverse_dict = {}
    last_valid_frame_id = -1

    min_mse_0 = 1 # just relative large number to start with
    ktime_ref_0 = 0
    frame_id_ref_0 = 0
    min_mse_1 = 1 # just relative large number to start with
    ktime_ref_1 = 0
    frame_id_ref_1 = 0

    last_frame_id = -1
    # getting the smallest rms event and treat it as golden
    with open(file_path, 'r') as fi:
        mod_index = 0        
        for line in fi:
            line_json = json.loads(line)
            ktime = line_json['ktime'] * 1000.0
            mtime = line_json['mtime'] * 1000.0
            valid = line_json['payload_present']
            mse = line_json['mean_squared_error']
            mod = line_json['frame_number'] % VICON_FPS

            if mod_index < 11:
                print 'mod = {}'.format(mod)
                mod_index = mod_index + 1
            #print 'mod = {}'.format(mod)
            if mod != 0:
                # Vicon processsing should take at most 4 frames:
                # dependent on Vicon tracker load: latency increases when unlabeled markers are present
                if mod > VICON_FPS - 5:
                    frame_id = line_json['frame_number'] + VICON_FPS - mod
                elif mod < 5:
                    frame_id = line_json['frame_number'] - mod
                else:
                    continue
            else:
                frame_id = line_json['frame_number']
            if frame_id == last_frame_id:
                continue
            last_frame_id = frame_id
            if mse < min_mse_0:
                min_mse_1 = min_mse_0
                ktime_ref_1 = ktime_ref_0
                frame_id_ref_1 = frame_id_ref_0
                min_mse_0 = mse
                ktime_ref_0 = ktime
                frame_id_ref_0 = frame_id
            elif mse < min_mse_1:
                min_mse_1 = mse
                ktime_ref_1 = ktime
                frame_id_ref_1 = frame_id
        if min_mse_1 == 1 or min_mse_0 == 1:
            assert 0, 'no more than two valid reference points found'
        frame_per_ktime_ref = (frame_id_ref_1 - frame_id_ref_0) / 1.0 / (ktime_ref_1 - ktime_ref_0)

    with open(file_path, 'r') as fi:
        last_frame_id = -1
        for line in fi:
            line_json = json.loads(line)
            ktime = line_json['ktime'] * 1000.0
            mtime = line_json['mtime'] * 1000.0
            valid = line_json['payload_present']
            ### sync comes when ahead of processed frame data from Vicon
            mod = line_json['frame_number'] % VICON_FPS
            if mod != 0:
                # Vicon processsing should take at most 4 frames:
                # dependent on Vicon tracker load: latency increases when unlabeled markers are present
                if mod > VICON_FPS - 5:
                    frame_id = line_json['frame_number'] + VICON_FPS - mod
                elif mod < 5:
                    frame_id = line_json['frame_number'] - mod
                else:
                    print 'Vicon sync {} is not within 5 frames of a multiple of {}' \
                        ': skip this frame'.format(line_json['frame_number'], VICON_FPS)
                    continue
            else:
                frame_id = line_json['frame_number']
            if frame_id == last_frame_id:
                continue
            last_frame_id = frame_id
            mse = line_json['mean_squared_error']
            assert valid == True, 'all events logged should be valid'
            # only when mse is smaller than tolerance
            # and either it is the first frame
            #     or this frame is a multiple of VICON_FPS apart
            #        and this multiple is less than FRAME_DROP_TOLERANCE
            ratio = 1
            if ktime != ktime_ref_0:
                frame_per_ktime = (frame_id - frame_id_ref_0) / 1.0 / (ktime - ktime_ref_0)
                ratio = frame_per_ktime / 1.0 / frame_per_ktime_ref
            if mse < MSE_TOLERANCE and \
               (ratio < 1.05 and ratio > 0.95) and \
               (last_valid_frame_id == -1 or (
                   ((frame_id - last_valid_frame_id) % VICON_FPS == 0) and \
                   ((frame_id - last_valid_frame_id) / VICON_FPS < FRAME_DROP_TOLERANCE) and \
                   ((frame_id - last_valid_frame_id) / VICON_FPS > 0))):
                logging.debug('last: {}, this: {}, reference_ratio: {} (should be close to 1)'.format(last_valid_frame_id, frame_id, ratio))
                if frame_id == last_valid_frame_id:
                    print 'warning: duplicate frame'
                    continue
                assert frame_id > last_valid_frame_id, 'frame num should be increasing'
                # use mtime from wearable (expected wearable time)
                assert frame_id % VICON_FPS == 0, 'frame id has to be a multiple of Vicon fps'
                dict[mtime] = frame_id + SYNC_DELAY_IN_FRAME
                reverse_dict[frame_id + SYNC_DELAY_IN_FRAME] = mtime
                last_valid_frame_id = frame_id

    assert len(dict.keys()) > 2, 'at least 2 valid frames captured'
    logging.info('Finished processing sync data')
    return dict, reverse_dict


# get a dictonary from frame id to a dictionary of object poses
# {frame_id_0: {object1: pose_of_object1, object2: pose_of_object2}}
def process_vicon(file_path, rig_object, wand_object = 'wand'):
    dict = {}
    empty_pose = np.array([[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,1]])
    with open(file_path, 'r') as fi:
        for line in fi:
            line_json = json.loads(line)
            frame_id = line_json['FrameNumber']
            # get 6dof pose for the objects of interest
            if rig_object in line_json['Bodies'] and wand_object in line_json['Bodies']:
                rig_pose = np.array(line_json['Bodies'][rig_object]['Transform44'])
                wand_pose = np.array(line_json['Bodies'][wand_object]['Transform44'])
                # only if both rig pose and wand pose are present (not 0) we capture this frame
                if (rig_pose - empty_pose).any() and (wand_pose - empty_pose).any():
                    assert frame_id not in dict, 'each frame should be unique'
                    dict[frame_id] = {wand_object: wand_pose, rig_object: rig_pose}
    logging.info('Finished processing vicon data')
    return dict

# extract world camera timestamp and index and annotate pose
def process_rigframes(file_paths, timestamp_to_frame_id, frame_id_to_poses,
                      source_path, output_path,
                      rig_object, wand_object = 'wand'):
    valid_frame_index = 1
    frame_index = 1
    for file_path in sorted(file_paths):
        assert file_path.endswith('rigframe_header_{0:06d}.yaml'.format(frame_index)), \
        'frame {} not found'.format(frame_index)
        with open(file_path) as fi:
            # extract world camera timestamp in wearable timestamp
            rigframe = yaml.load(fi)
            rigframe['index'] = valid_frame_index
            timestamps = [sensor['timestamp'] for sensor in rigframe['timestamps']]
            assert len(set(timestamps)) == 1, 'world cameras are not synced: {}'.format(rigframe)
            timestamp = timestamps[0]
            poses_interp = interpolate_vicon_pose_from_wearable_timestamp(
                timestamp, timestamp_to_frame_id, frame_id_to_poses)
            if rig_object not in poses_interp or wand_object not in poses_interp:
                logging.warn('{} and {} should be present after interpolation '
                             'either this timestamp does not have corresponding sync '
                             'or the two vicon frames that get interpolated from '
                             'does not have rig and wand pose'.format(
                    rig_object, wand_object))
                frame_index += 1
                continue
            else:
                origin_T_rig_pose = poses_interp[rig_object]
                rig_T_origin_pose = np.linalg.inv(origin_T_rig_pose)
                origin_T_wand_pose = poses_interp[wand_object]
                rig_T_wand_pose = np.dot(rig_T_origin_pose, origin_T_wand_pose)

                # write to yaml rigframe file (part 1 of 2: all the original content)
                rigframe.pop('transformations')
                yaml_output_path = os.path.join(output_path,
                                                'rigframe_header_{0:06d}.yaml'.format(
                                                    valid_frame_index))
                with open(yaml_output_path, 'w') as fo:
                    yaml.dump(rigframe, fo, Dumper=YamlDumper, default_flow_style=False)
                # write to yaml rigframe file (part 2 of 2: rig_T_wand_pose)
                out_dict = {}
                out_dict['transformations']=[{'dst': 'vicon_rig',
                                              'src': 'vicon_board',
                                              'T': [rig_T_wand_pose[0, :].tolist(),
                                                    rig_T_wand_pose[1, :].tolist(),
                                                    rig_T_wand_pose[2, :].tolist(),
                                                    rig_T_wand_pose[3, :].tolist()]}]
                with open(yaml_output_path, 'a') as fo:
                    yaml.dump(out_dict, fo, Dumper=YamlDumper)

                # copy side/world left/right images
                for sw in ['side', 'world']:
                    for lr in ['left', 'right']:
                        copyfile(os.path.join(source_path,
                                              sw + '_' + lr + \
                                              '_{0:06d}.png'.format(
                                                  frame_index)),
                                 os.path.join(output_path,
                                              sw + '_' + lr + \
                                              '_{0:06d}.png'.format(
                                                  valid_frame_index)))
                valid_frame_index += 1
                frame_index += 1

    # copy sensors.yaml
    copyfile(os.path.join(source_path, 'sensors.yaml'),
             os.path.join(output_path, 'sensors.yaml'))

    # write the log.txt file
    with open(os.path.join(output_path, 'log.txt'), 'w') as fo:
        for id in range(1, valid_frame_index):
            fo.write(str(id) + ' rigframe_header_{0:06d}.yaml\n'.format(id))
    logging.info('Finished writing aligned rigframes to {}'.format(output_path))

# use wearable timestamp to first lookup the Vicon frame id before and after
# get the interpolated Vicon frame id
# use interpolated frame id to lookup for closest two pose objects
# generate an interpolated pose object
def interpolate_vicon_pose_from_wearable_timestamp(timestamp,
                                                   timestamp_to_frame_id,
                                                   frame_id_to_poses):
    # search through the sync timestamps which we have vicon frame id
    sorted_sync_timestamps = sorted(timestamp_to_frame_id.keys())
    insert_index = np.searchsorted(sorted_sync_timestamps, timestamp)
    if timestamp < sorted_sync_timestamps[0] - TIMESTAMP_EXTRAPOLATE_LIMIT \
       or timestamp > sorted_sync_timestamps[-1] + TIMESTAMP_EXTRAPOLATE_LIMIT:
        logging.warning('timestamp {} is not in between two vicon sync events'.format(
            timestamp))
        return {}
    elif insert_index == 0:
        left_timestamp = sorted_sync_timestamps[0]
        right_timestamp = sorted_sync_timestamps[-1]
        alpha = (left_timestamp - timestamp) * 1. / (right_timestamp - left_timestamp)
        assert alpha >= 0
        left_frame_id = timestamp_to_frame_id[left_timestamp]
        right_frame_id = timestamp_to_frame_id[right_timestamp]
        best_frame_id = left_frame_id * (1 + alpha) - right_frame_id * alpha
    elif insert_index == len(sorted_sync_timestamps):
        left_timestamp = sorted_sync_timestamps[0]
        right_timestamp = sorted_sync_timestamps[-1]
        alpha = (timestamp - right_timestamp) * 1. / (right_timestamp - left_timestamp)
        assert alpha >= 0
        left_frame_id = timestamp_to_frame_id[left_timestamp]
        right_frame_id = timestamp_to_frame_id[right_timestamp]
        best_frame_id = right_frame_id * (1 + alpha) - left_frame_id * alpha
    else:
        # get the sync timestamp before and after the timestamp of interest
        left_sync_timestamp = sorted_sync_timestamps[insert_index-1]
        right_sync_timestamp = sorted_sync_timestamps[insert_index]
        alpha = (timestamp - left_sync_timestamp) * 1. / (
            right_sync_timestamp - left_sync_timestamp)

        # get the best fit frame id
        left_frame_id = timestamp_to_frame_id[left_sync_timestamp]
        right_frame_id = timestamp_to_frame_id[right_sync_timestamp]
        logging.debug('timestamp {} is between {}: {} and {}: {}'.format(
            timestamp,
            left_sync_timestamp,
            left_frame_id,
            right_sync_timestamp,
            right_frame_id))
        best_frame_id = left_frame_id * (1 - alpha) + right_frame_id * alpha

    # logging.debug('best frame found is {}'.format(best_frame_id))

    # from best_frame_id, get the pose object from the frame before and after
    sorted_frame_ids = sorted(frame_id_to_poses)
    insert_index_for_pose = np.searchsorted(sorted_frame_ids, best_frame_id)
    left_frame_id_for_pose = sorted_frame_ids[insert_index_for_pose-1]
    right_frame_id_for_pose = sorted_frame_ids[insert_index_for_pose]

    if insert_index_for_pose == 0 \
       or insert_index_for_pose == len(sorted_frame_ids):
        return {}

    alpha_poses = (best_frame_id - left_frame_id_for_pose) * 1. / (
        right_frame_id_for_pose - left_frame_id_for_pose)
    left_poses = frame_id_to_poses[left_frame_id_for_pose]
    right_poses = frame_id_to_poses[right_frame_id_for_pose]

    # get the pose object before and after
    interpolated_poses = {}
    for obj in left_poses.keys():
        assert obj in right_poses, '{} not present in the two pose objects'.format(obj)
        interpolated_poses[obj] = interpolate_pose(left_poses[obj], right_poses[obj],
                                                  alpha_poses)
    return interpolated_poses

def interpolate_wearable_timestamp_from_vicon_frame(frame_id_to_timestamp, frame_id):
    sorted_frame_ids = sorted(frame_id_to_timestamp.keys())
    insert_index = np.searchsorted(sorted_frame_ids, frame_id)
    #make sure the frame_id is within the range
    if frame_id < sorted_frame_ids[0] - FRAME_EXTRAPOLATE_LIMIT or frame_id > sorted_frame_ids[-1] + FRAME_EXTRAPOLATE_LIMIT:
        return None
    elif insert_index == 0:
        left_frame_id = sorted_frame_ids[0]
        right_frame_id = sorted_frame_ids[-1]
        alpha = (left_frame_id - frame_id) * 1. / (right_frame_id - left_frame_id)
        assert alpha >= 0
        left_timestamp = frame_id_to_timestamp[left_frame_id]
        right_timestamp = frame_id_to_timestamp[right_frame_id]
        best_timestamp = left_timestamp * (1 + alpha) - right_timestamp * alpha
    elif insert_index == len(sorted_frame_ids):
        left_frame_id = sorted_frame_ids[0]
        right_frame_id = sorted_frame_ids[-1]
        alpha = (frame_id - right_frame_id) * 1. / (right_frame_id - left_frame_id)
        assert alpha >= 0
        left_timestamp = frame_id_to_timestamp[left_frame_id]
        right_timestamp = frame_id_to_timestamp[right_frame_id]
        best_timestamp = right_timestamp * (1 + alpha) - left_timestamp * alpha
    else:
        # get the sync timestamp before and after the timestamp of interest
        left_frame_id = sorted_frame_ids[insert_index-1]
        right_frame_id = sorted_frame_ids[insert_index]
        # print 'left frame: {}; right frame: {}'.format(left_frame_id, right_frame_id)
        alpha = (frame_id - left_frame_id) * 1. / (
            right_frame_id - left_frame_id)
        assert alpha <= 1 and alpha >= 0
        left_timestamp = frame_id_to_timestamp[left_frame_id]
        right_timestamp = frame_id_to_timestamp[right_frame_id]
        # print 'left: {}; right: {}'.format(left_timestamp, right_timestamp)
        best_timestamp = left_timestamp * (1 - alpha) + right_timestamp * alpha
        # print 'left: {}; right: {}; interp: {}'.format(left_timestamp, right_timestamp, best_timestamp)

    return best_timestamp


# interpolate two 6dof poses
# pose1 and pose2 are 4x4 numpy matrices
# 0 <= alpha <=1
def interpolate_pose(pose1, pose2, alpha):
    assert alpha >=0 and alpha <= 1

    # interpolating rotation component
    q1 = Quaternion(matrix = pose1)
    q2 = Quaternion(matrix = pose2)
    q_interp = Quaternion.slerp(q1, q2, alpha)
    rot_interp = q_interp.transformation_matrix

    # interpolating translation component
    t1 = pose1[0:3, 3]
    t2 = pose2[0:3, 3]
    trans_interp = t1 * (1. - alpha) + t2 * alpha
    rot_interp[0:3, 3] = rot_interp[0:3, 3] + trans_interp
    return rot_interp

# custom yaml dumper to increase indentation
class YamlDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(YamlDumper, self).increase_indent(flow, False)

def main(args):
    parser = argparse.ArgumentParser('process')
    parser.add_argument('mode', help = 'either \'vicon_calib\' or \'vicon_timestamp_align\'',
                        choices = ['vicon_calib', 'vicon_timestamp_align'])
    parser.add_argument('--vicon', help = 'Vicon data in JSON', required = True)
    parser.add_argument('--sync', help = 'decoded sync output', required = True)
    parser.add_argument('--output', help = 'output directory', required = True)
    # only in 'vicon_calib' mode
    parser.add_argument('--calib', help = 'source of calibration')
    parser.add_argument('--rig_object', help = 'name of rig object')
    # only in 'vicon_timestamp_align' mode
    parser.add_argument('--from_time', type=int, help = 'timestamp from')
    parser.add_argument('--to_time', type=int, help = 'timestamp to')

    args = parser.parse_args(args)

    # uncomment the following to printout debug msg
    # logging.basicConfig(level = logging.DEBUG)
    assert os.path.isfile(args.sync), 'decoded sync output {} does not exist'.format(args.sync)
    assert os.path.isfile(args.vicon), 'Vicon data {} does not exist'.format(args.vicon)

    if args.mode == 'vicon_calib':
        assert args.rig_object != None, 'please specify rig_object'
        assert args.calib != None, 'please specify source of calibration'
        assert os.path.isdir(args.calib), 'Vicon chessboard data {} does not exist'.format(args.calib)

        # get a dictionary from wearable time to vicon frame id
        wearable_time_to_vicon_frame, vicon_frame_to_wearable_time = process_sync(args.sync)

        # get a dictonary from frame id to a dictionary of object poses
        vicon_frame_to_object_poses = process_vicon(args.vicon, args.rig_object)

        # parse vicon_chessboard_data directory for all the frame timestamp
        rigframe_yamls = sorted(glob.glob('{}/rigframe*.yaml'.format(args.calib)))
        assert len(rigframe_yamls) != 0, 'no rigframe_*.yaml found in vicon data dir {}'.format(args.calib)
        assert len(rigframe_yamls) > 0, 'at least 20 images should be captured for calibration'

        # generate output directory
        dirs = [os.path.join(args.output, 'calibration', 'debug'),
                os.path.join(args.output, 'calibration', 'front_chessboard')]
        for dir in dirs:
            if not os.path.isdir(dir):
                os.makedirs(dir)

        # process rigframes and search for the corresponding pose
        process_rigframes(rigframe_yamls, wearable_time_to_vicon_frame, vicon_frame_to_object_poses,
                          source_path = args.calib,
                          output_path = os.path.join(args.output, 'calibration', 'front_chessboard'),
                          rig_object = args.rig_object)

    elif args.mode == 'vicon_timestamp_align':
        assert args.from_time != None, 'please specify device timestamp to start with'
        assert args.to_time != None, 'please specify device timestamp to end until'

        # get a dictionary from wearable time to vicon frame id
        wearable_time_to_vicon_frame, vicon_frame_to_wearable_time = process_sync(args.sync)

        # prepare output directory
        basename = os.path.splitext(os.path.basename(args.vicon))[0]
        if not os.path.isdir(args.output):
            os.makedirs(args.output)
        output_file = os.path.join(args.output, basename + '_interpolated.txt')

        with open(args.vicon, 'r') as fi, open(output_file, 'w') as fo:
            for line in fi:
                line_json = json.loads(line)
                frame_id = line_json['FrameNumber']
                timestamp = interpolate_wearable_timestamp_from_vicon_frame(vicon_frame_to_wearable_time, frame_id)
                if timestamp is not None and \
                   (timestamp > args.from_time and timestamp < args.to_time):
                    line_json['WearableTimestamp'] = timestamp
                    json.dump(line_json, fo)
                    fo.write('\n')

if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
