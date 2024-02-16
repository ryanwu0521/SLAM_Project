
import os
import platform
import json
import subprocess
import argparse

def run_linux_system(args):
    python_ver = platform.python_version_tuple()
    if int(python_ver[1]) > 8:
        print('Not yet tested for beyond python3.8')
    elif int(python_ver[1]) == 8:
        import lsb_release
        if lsb_release.get_distro_information()['RELEASE'] != '20.04':
            print('Only Ubuntu 20.04 supported')
            exit()
    else:
        print('Not supported for python before 3.8')
        exit()
    
    if os.path.isfile("config.json"):
        f = open('config.json')
        configs = json.load(f)
        isaac_path = configs['isaac_path']
    else:
        print(os.getcwd()+"/config.json not found, searching default isaac install path")

        isaac_path = os.environ['HOME']+'/.local/share/ov/pkg'
        if not os.path.isdir(isaac_path):
            print('Default install path not found')
            exit()
        dir_list = os.listdir(isaac_path)
        isaac_list = [s for s in dir_list if "isaac" in s]
        if len(isaac_list) == 0:
            print('No Isaac install found in default path. Omniverse likely installed but no Isaac installed')
        elif len(isaac_list) > 1:
            print('Multiple Isaac versions found, using newest version ' + isaac_list[0])
        
        isaac_path = isaac_path+"/"+isaac_list[0]

    if not "2023" in isaac_path:
        print("Warning, this has only been tested for Isaac 2023")

    file_path = os.getcwd() + "/" + args.file_path
    subprocess.Popen(["bash", isaac_path+"/python.sh", file_path])
    exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--file_path", type=str,default="src/code/main.py",help="path to python script to run in isaac")
    args = parser.parse_args()
    # print(os.getcwd()+"/"+args.file_path)
    python_ver = platform.python_version_tuple()
    if int(python_ver[0]) != 3:
        print("Only supported for python3")
        exit()

    system = platform.system()
    if system == 'Linux':
        run_linux_system(args)
    else:
        print("System not yet supported")
        exit()
