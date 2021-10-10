import argparse
import os
from converter import Converter
import time

name2const = {
    'RGB332': Converter.FLAG.CF_TRUE_COLOR_332,
    'RGB565': Converter.FLAG.CF_TRUE_COLOR_565,
    'RGB565SWAP': Converter.FLAG.CF_TRUE_COLOR_565_SWAP,
    'RGB888': Converter.FLAG.CF_TRUE_COLOR_888,
    'alpha_1': Converter.FLAG.CF_ALPHA_1_BIT,
    'alpha_2': Converter.FLAG.CF_ALPHA_2_BIT,
    'alpha_4': Converter.FLAG.CF_ALPHA_4_BIT,
    'alpha_8': Converter.FLAG.CF_ALPHA_8_BIT,
    'indexed_1': Converter.FLAG.CF_INDEXED_1_BIT,
    'indexed_2': Converter.FLAG.CF_INDEXED_2_BIT,
    'indexed_4': Converter.FLAG.CF_INDEXED_4_BIT,
    'indexed_8': Converter.FLAG.CF_INDEXED_8_BIT,
    'raw': Converter.FLAG.CF_RAW,
    'raw_alpha': Converter.FLAG.CF_RAW_ALPHA,
    'raw_chroma': Converter.FLAG.CF_RAW_CHROMA,
    'true_color': Converter.FLAG.CF_TRUE_COLOR,
    'true_color_alpha': Converter.FLAG.CF_TRUE_COLOR_ALPHA,
    'true_color_chroma': Converter.FLAG.CF_TRUE_COLOR_CHROMA,
}


def CheckAllowed(filepath):
    path_split = os.path.splitext(filepath)
    suffix: str = path_split[-1]

    return suffix.lower() in ['.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tga', '.gif', '.bin']


def ConvOneFile(filepath, f, cf, ff: str, dith, out_path=''):
    path_split = os.path.split(filepath)
    root_path = path_split[0]
    name_split = os.path.splitext(path_split[-1])
    name = name_split[0]
    conv = Converter(filepath, name, dith, name2const[f])

    c_arr = ''
    if f in ['true_color', 'true_color_chroma']:
        conv.convert(name2const[cf])
        c_arr = conv.format_to_c_array()
    elif f in ['true_color_alpha']:
        conv.convert(name2const[cf], 1)
        c_arr = conv.format_to_c_array()
    else:
        conv.convert(name2const[f])

    file_conf = {
        'C': {
            True: os.path.join(out_path, f"{name}.c"),
            False: os.path.join(root_path, f"{name}.c"),
            'mode': 'w'},
        'BIN': {
            True: os.path.join(out_path, f"{name}.bin"),
            False: os.path.join(root_path, f"{name}.bin"),
            'mode': 'wb'}
    }

    with open(file_conf[ff][bool(out_path)], file_conf[ff]['mode']) as fi:
        res = conv.get_c_code_file(name2const[f], c_arr) if ff == 'C' else conv.get_bin_file(name2const[f])
        fi.write(res)
    return "SUCCESS"


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', type=str, nargs="+",
                        help='images dir paths (or file paths) you wanna convert')
    parser.add_argument('-f', '-format', type=str, default='true_color_alpha',
                        choices=['true_color', 'true_color_alpha', 'true_color_chroma',
                                 'indexed_1', 'indexed_2', 'indexed_4', 'indexed_8',
                                 'alpha_1', 'alpha_2', 'alpha_4', 'alpha_8',
                                 'raw', 'raw_alpha', 'raw_chroma'],
                        help='converted file format: \n'
                             'true_color, true_color_alpha, true_color_chroma, '
                             'indexed_1, indexed_2, indexed_4, indexed_8, '
                             'alpha_1, alpha_2, alpha_4, alpha_8, '
                             'raw, raw_alpha, raw_chroma. The default is: true_color_alpha')
    parser.add_argument('-cf', '-color-format', type=str, default='RGB565',
                        choices=['RGB332', 'RGB565', 'RGB565SWAP', 'RGB888'],
                        help='converted color format: RGB332, RGB565, RGB565SWAP, RGB888')
    parser.add_argument('-ff', '-file-format', type=str, default='C',
                        choices=['C', 'BIN'],
                        help='converted file format: C(*.c), BIN(*.bin)')
    parser.add_argument('-o', '-output-filepath', type=str, default='',
                        help='output file path. if not set, it will saved in the input dir')
    parser.add_argument('-r', action="store_const", const=True,
                        help='convert files recursively')
    parser.add_argument('-d', action="store_const", const=True,
                        help='need to dith')
    args = parser.parse_args()

    # Namespace(cf='RGB888', f='true_color', ff='C', filepath=['fuckme.jpg'], o='/')

    file_count = 0
    failed_pic_paths = []
    for path in args.filepath:
        if os.path.isdir(path):
            for root, dirs, files in os.walk(path):
                if not args.r and root != path:
                    break
                for item in files:
                    abs_path = os.path.abspath(os.path.join(root, item))
                    if not CheckAllowed(abs_path): continue
                    print(f'{file_count:<5} {abs_path} START', end='')
                    t0 = time.time()

                    try:
                        conv_rtn = ConvOneFile(abs_path, args.f, args.cf, args.ff, args.d, args.o)
                        if conv_rtn == "SUCCESS":
                            file_count += 1
                            print('\b' * 5 + 'FINISHED', end='')
                        elif conv_rtn == "NOT ALLOWED":
                            print('\b' * 5, end='')
                    except Exception as e:
                        print('\b' * 5, e, end='')
                        failed_pic_paths.append(abs_path)
                    print(f' {(time.time() - t0) * 1000} ms')
        else:
            print(f'{file_count:<5} {path} START', end='')
            t0 = time.time()

            try:
                conv_rtn = ConvOneFile(path, args.f, args.cf, args.ff, args.d, args.o)
                if conv_rtn == "SUCCESS":
                    file_count += 1
                    print('\b' * 5 + 'FINISHED', end='')
                elif conv_rtn == "NOT ALLOWED":
                    print('\b' * 5, end='')
            except Exception as e:
                print('\b' * 5, e, end='')
                failed_pic_paths.append(path)
            print(f' {(time.time() - t0) * 1000} ms')
    print()
    print(f"Convert Complete. Total convert {file_count} file(s).")
    print()
    print("Failed File List:")
    for path in failed_pic_paths:
        print(path)
