#!/usr/bin/python

import argparse
import subprocess
import time

# Include libraries from mlx_calibration
try:
    # When loading from bin in the new build
    import __ml_env__
except:
    pass

from lib_io import *
from lib_user import *
from util_terminal import *

TEMP_FOLDER_NAME="wearable_prop_calibration_tmp"
TEMP_DIR_PP = "/data/" + TEMP_FOLDER_NAME
TEMP_DIR_SERDES = "/tmp/" + TEMP_FOLDER_NAME

DEPTH_EEPROM_CALIBRATION_FILENAME= 'wearable_depth_calibration.rig'

batch_size_ = 1
via_powerpack_ = True

def get_temperary_folder_full_path():
    if via_powerpack_:
        temp_dir = TEMP_DIR_PP
    else:
        temp_dir = TEMP_DIR_SERDES
    return temp_dir

def run_command(cmd, wait_seconds):
    subprocess.call(cmd, shell=True)
    time.sleep(wait_seconds)
    #print make_bold("Excecuting cmd: " + cmd)

def config_pss():
    # each item in mldb_cmds is a pair of command and wait time
    mldb_cmds = []

    #mldb_cmds.append(("mldb reboot", 30))
    mldb_cmds.append(("mldb wait-for-device", 1))
    mldb_cmds.append(("mldb root", 1))
    mldb_cmds.append(("mldb remount", 1))
    mldb_cmds.append(("mldb shell stop universe", 1))
    mldb_cmds.append(("mldb shell stop magicflinger", 1))

    print make_bold("start to config pss...")
    for item in mldb_cmds:
        cmd = item[0]
        wait_time = item[1]
        run_command(cmd, wait_time)

    print make_bold("finish pss configuration.")

def initialize_pp():
    config_pss()

    cmd = "mldb shell rm -rf " + get_temperary_folder_full_path()
    run_command(cmd, 5)

    cmd = "mldb shell mkdir " + get_temperary_folder_full_path()
    run_command(cmd, 5)

    cmd = "mldb shell stop graph_pss";
    run_command(cmd, 2)

def initialize_serdes():
    cmd = "rm -rf " + TEMP_DIR_SERDES
    run_command(cmd, 5)
    cmd = "mkdir " + TEMP_DIR_SERDES
    run_command(cmd, 1)

def initialize():
    if via_powerpack_:
        initialize_pp()
    else:
        initialize_serdes()

def extract_depth_cal_from_rom():
    # Extract depth calibration rig file to temp folder.
    tmp_dir = get_temperary_folder_full_path()
    if via_powerpack_:
        cmd = 'mldb shell depth_regulator_gadget -fetch ' + tmp_dir
        run_command(cmd, 60)
    else:
        cmd = 'depth_regulator_gadget -fetch ' + tmp_dir
        run_command(cmd, 60)

def generate_cmd_for_snapshot_wearable_prop(data_mode, output_folder):
    if via_powerpack_:
        command = "mldb shell mal_test_wearable "
    else:
        command = "graph_test_wearable "

    assert (batch_size_ >= 1)
    assert (batch_size_ <= 10)
    cmd_capture_args = get_data_capture_cmd_args_from_data_mode(data_mode)
    cmd = command + "-outpath=" + output_folder + cmd_capture_args
    return cmd

def generate_cmd_for_snapshot_vicon_calibration(output_folder):
    if via_powerpack_:
        command = "mldb shell mal_test_wearable "
    else:
        command = "graph_test_wearable "

    assert (batch_size_ >= 1)
    assert (batch_size_ <= 10)
    cmd_capture_args = " -qc_samples=" + str(batch_size_) +  " -synced=1 -save_mode 1"
    cmd = command + "-outpath=" + output_folder + cmd_capture_args
    return cmd

def get_data_capture_cmd_args_from_data_mode(data_mode):
    """ Prepare args appropriately for the app main_test_wearable.cpp given the input data mode.

    Args:
        data_mode: (str) the data mode for the capture.
    """
    if data_mode == "wearable_prop":
        cmd_capture_args = " -ec_samples=" + str(batch_size_) + " -qc_samples=" + str(batch_size_) \
                         + " -dc_samples=" + str(batch_size_) + " -vc_samples=" + str(batch_size_) + " -synced=1 -save_mode=1"
        cmd_capture_args = cmd_capture_args + " --illumination=100 --eyes_gain=5.0 --eyes_exposure=850"
    else:
        assert data_mode == "vicon_chessboard_data"
        cmd_capture_args = " -qc_samples=" + str(batch_size_) + \
                         + " -dc_samples=" + str(batch_size_) + " -vc_samples=" + str(batch_size_) +" -synced=1 -save_mode=1"
    return cmd_capture_args

def collect_rigframe(data_mode, frame_index):
    assert (frame_index > 0)
    temp_out_frame_folder = get_temperary_folder_full_path()

    if data_mode == "wearable_prop":
        cmd = generate_cmd_for_snapshot_wearable_prop(data_mode, temp_out_frame_folder)
        run_command(cmd, 1)
    elif data_mode == "vicon_chessboard_data":
        cmd = generate_cmd_for_snapshot_vicon_calibration(temp_out_frame_folder)
        print cmd
        run_command(cmd, 1)
    else:
        print make_bold("Wrong input: please input a valid data mode!")

def clean_temp_folders():
    if via_powerpack_:
        clean_temp_folders_pp()
    else:
        clean_temp_folders_serdes()

def clean_temp_folders_pp():
    cmd = "mldb shell rm  " + get_temperary_folder_full_path() + "/*.pgm"
    run_command(cmd, 1)

    cmd = "mldb shell rm  " + get_temperary_folder_full_path() + "/D*.bin"
    run_command(cmd, 1)

    cmd = "mldb shell rm  " + get_temperary_folder_full_path() + "/V*.bin"
    run_command(cmd, 1)

    cmd = "mldb shell rm  " + get_temperary_folder_full_path() + "/timestamps.txt"
    run_command(cmd, 1)

def clean_temp_folders_serdes():
    cmd = "rm  " + get_temperary_folder_full_path() + "/*.pgm"
    run_command(cmd, 1)

    cmd = "rm  " + get_temperary_folder_full_path() + "/D*.bin"
    run_command(cmd, 1)

    cmd = "rm  " + get_temperary_folder_full_path() + "/timestamps.txt"
    run_command(cmd, 1)

def mv_data_from_temp(pc_cache_dir, temp_folder_full_path, temp_folder_name):
    # The flow of data moving is as following:
    # 1. pull the folder temp_folder_full_path from power pack to current directory.
    # 2. copy all the useful files in temp_folder_name (copied from step 1) sitting to pc_cache_dir
    # 3. clean up the folder in temp_folder_full_path of powerpack.
    local_temp_path = os.getcwd() + "/" + temp_folder_name
    if os.path.isdir(local_temp_path):
        shutil.rmtree(local_temp_path)

    # mldb pull only support convenient pull the folder
    if via_powerpack_:
        copy_command = "mldb pull " + temp_folder_full_path + " ."
    else:
        copy_command = "cp -r " + temp_folder_full_path + " ."

    # pull the data folder from temp_folder_full_path to local disk.
    run_command(copy_command, 1)

    wait_time = 0
    mv_command = "mv  " + local_temp_path + "/*.pgm " + pc_cache_dir
    run_command(mv_command, wait_time)
    mv_command = "mv  " + local_temp_path + "/D*.bin " + pc_cache_dir
    run_command(mv_command, wait_time)
    mv_command = "mv  " + local_temp_path + "/V*.bin " + pc_cache_dir
    run_command(mv_command, wait_time)
    mv_command = "mv  " + local_temp_path + "/timestamps.txt " + pc_cache_dir
    run_command(mv_command, wait_time)

    clean_temp_folders()

def mv_depth_calibration_data_from_temp(pc_cache_dir, temp_folder_full_path, temp_folder_name):
    # The flow of data moving is as following:
    # 1. pull the folder temp_folder_full_path from power pack to current directory.
    # 2. copy depth calibration data sitting in temp_folder_name (copied from step 1) to pc_cache_dir
    local_temp_path = os.getcwd() + "/" + temp_folder_name
    if os.path.isdir(local_temp_path):
        shutil.rmtree(local_temp_path)

    # mldb pull only support convenient pull the folder
    if via_powerpack_:
        copy_command = "mldb pull " + temp_folder_full_path + " ."
    else:
        copy_command = "cp -r " + temp_folder_full_path + " ."

    # pull the data folder from temp_folder_full_path to local disk.
    run_command(copy_command, 1)

    wait_time = 0
    mv_command = "cp  " + local_temp_path + "/" + DEPTH_EEPROM_CALIBRATION_FILENAME + ' ' + pc_cache_dir
    run_command(mv_command, wait_time)

    # merge loaded depth intrinsics calibration results into consumable format.
    merge_command = "add_depth_calibratio " + pc_cache_dir + " " + pc_cache_dir
    run_command(mv_command, wait_time)


def is_valid_wd_id(wd_id):
    if wd_id is None or " " in wd_id or len(wd_id) > 20:
        return False
    return True

def main(cache_directory, data_mode):
    """
    Args:
        cache_directory: (str) Full path to cache directory.
    """

    # Prepare a clean disk for both powerpack and local pc.
    if not os.path.isdir(cache_directory):
        command = "mkdir " + cache_directory
        run_command(command, 1)

    assert data_mode == "wearable_prop" or data_mode == "vicon_chessboard_data"

    temp_folder_full_path = get_temperary_folder_full_path()
    temp_folder_name = TEMP_FOLDER_NAME

    initialize()

    if data_mode == "wearable_prop":
        # Download depth data from eeprom if not exists in cache directory.
        if not os.path.isfile(os.path.join(cache_directory, DEPTH_EEPROM_CALIBRATION_FILENAME)):
            extract_depth_cal_from_rom()
            mv_depth_calibration_data_from_temp(cache_directory, \
                                            temp_folder_full_path, \
                                            temp_folder_name)

    print make_bold("Starting " + data_mode + " data collection ...")
    # Collect data for all cameras on wearable.
    capture_one_more = True
    frame_index = 0

    while capture_one_more:
        frame_index = frame_index + 1
        # collect one rigframe via mldb commands, the data would be stored in path specified by temp_folder_full_path.
        collect_rigframe(data_mode, frame_index)
        mv_data_from_temp(cache_directory, temp_folder_full_path, temp_folder_name)

	print('before reformat')
        # reformat the raw data collecte via malles into calibration consistent formats.
        cmd = "reformat_malleus_raw_data --cache " + cache_directory + " --mode " + data_mode + " --timestamps_log " + cache_directory + "/timestamps.txt" + " --snapshot_folder " + cache_directory + " -qc 1"
        run_command(cmd, 0)
	print('after reformat')

        # show the latest rigframe.
        cmd = "visualize_rigframe -cache " + cache_directory + " -index -1 " + "-mode " + data_mode
        run_command(cmd, 0)

        print make_bold(str(frame_index) + " rigframe was captured.")
        capture_one_more = ask_user_yes_no("Would you like to capture another frame?")

    print make_bold("Data collection is done. Copying data from temp folder to " + cache_directory + "!")

    # copy data from powerpack to local cache directory.
    #clean_temp_folders()

# Main
'''
This python file would assist collecting wearable data for PEQ devices.
It supports both serdes and powerpack connections.
TODO (ljiang): Add detection visualization for each step's captured rigframe.
'''
if __name__ == "__main__":
    # Define command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--powerpack", \
        help="The flag enable data collection via powerpack . ", \
        default="1")

    parser.add_argument("--batchsize", \
        help="Number of continuous images to capture (per sensor), batchsize can be any integer between 1 and 10.", \
        default="1")

    parser.add_argument('--cache', \
        help="Full path to the directory to store data. " + \
             "If directory exists, resumes calibration data collection from previous run. " + \
             "If not, calibration will be stored in the specifed directory.")

    parser.add_argument('--mode', \
        help="Data mode for calibration. " + \
             "Currently only supports the following modes :" + \
             "1. wearable_prop 2. vicon_chessboard_data.")

    # Parse
    args = parser.parse_args()
    start_time = time.time()
    sz_via_powerpack = args.powerpack
    assert sz_via_powerpack == "0" or sz_via_powerpack == "1"
    via_powerpack_ = sz_via_powerpack == "1"

    batch_size_ = int(args.batchsize)
    assert (batch_size_ >= 1)
    assert (batch_size_ <= 10)

    if via_powerpack_:
        print make_bold("Collect data via powerpack. ")
    else:
        print make_bold("Collect data via serdes. ")

    main(args.cache, args.mode)
    end_time = time.time() - start_time
    print make_bold("Total time takes: " + str(end_time / 60) + "min.")
