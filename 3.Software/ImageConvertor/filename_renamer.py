import argparse
import os
import time


def CheckAllowed(filepath):
    path_split = os.path.splitext(filepath)
    suffix: str = path_split[-1]

    return suffix.lower() in ['.bin']


def ConvOneFile(filepath, target_name):
    try:
        root_path = os.path.dirname(filepath)
        target_file_path = os.path.join(root_path, target_name)
        os.rename(filepath, target_file_path)
    except Exception as e:
        print(e)
        return 'NOT ALLOWED'
    return 'SUCCESS'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', type=str, nargs="+",
                        help='images dir paths (or file paths) you wanna convert')
    parser.add_argument('-o', '-output-filename', type=str,
                        help='output file name.')
    args = parser.parse_args()

    file_count = 0
    for path in args.filepath:
        if os.path.isdir(path):
            for root, dirs, files in os.walk(path):
                # if root == path:
                #     break
                for item in files:
                    abs_path = os.path.abspath(os.path.join(root, item))
                    if not CheckAllowed(abs_path): continue
                    print(f'{file_count:<5} {abs_path} START', end='')
                    t0 = time.time()
                    conv_rtn = ConvOneFile(abs_path, args.o)
                    if conv_rtn == "SUCCESS":
                        file_count += 1
                        print('\b' * 5 + 'FINISHED', end='')
                    elif conv_rtn == "NOT ALLOWED":
                        print('\b' * 5, end='')
                    print(f' {(time.time() - t0) * 1000} ms')
    print()
    print(f"Convert Complete. Total rename {file_count} file(s).")
