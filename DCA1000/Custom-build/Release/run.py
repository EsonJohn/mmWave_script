import os, sys

def parameter_hint():
    print("Parameters format:\n\
                python run.py [DCA_cfg_file] [file_prefix] -b [b1]...[bn] -r [r1]...[rm]\n\
            e.g., python run.py cf.json eson -b 6 5 5 5 -r 1 1 2 1\n\
                Note: DO NOT add directory to file_prefix, default to ./Data directory.")

def check_parameter(argv):
    is_legal = True
    if len(sys.argv) < 7 \
            or '-b' not in argv \
            or '-r' not in argv:
        parameter_hint()
        sys.exit()
    i_b = argv.index('-b')
    i_r = argv.index('-r')
    num_b = i_r - i_b - 1
    num_r = len(argv) - i_r - 1
    if num_b < 0 or num_b != num_r:
        parameter_hint()
        sys.exit()
    _, _, b, r = dump_parameter(argv)
    if b < r:
        print("Parameter error, '-b' should be no less than '-r'")
        parameter_hint()
        sys.exit()


def dump_parameter(argv):
    cfg_file = argv[1]
    file_prefix = argv[2]
    b_all = argv[argv.index('-b')+1:argv.index('-r')]
    r_all = argv[argv.index('-r')+1:]
    b_all = [int(i) for i in b_all]
    r_all = [int(i) for i in r_all]
    return cfg_file, file_prefix, b_all, r_all

# >>>> Program entrance
# parameter_hint()

# Parameter validation
check_parameter(sys.argv)

# Parameter extraction
cfg_file, file_prefix, b_all, r_all = dump_parameter(sys.argv)

# Connect and config DCA1000
print("Now configuring DCA1000...")
os.system("DCA1000EVM_CLI_Control.exe fpga cf.json")

# Record data
print("Now recording data...")
b_tmp = r_all
while b_tmp <= b_all:
    file_name = file_prefix
    for idx in range(0, len(b_tmp)):
        file_name = file_name + '-' + str(b_tmp[idx])
    
    print("\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Now recording " + file_name)
    os.system("DCA1000EVM_CLI_Record.exe start_record cf.json " + file_name)
    # input("Press Enter to continue...")
    
    # Increment file prefix
    if b_tmp == b_all:
        break
    for idx in range(len(b_tmp)-1,-1,-1):
        b_tmp[idx] = b_tmp[idx] + 1
        if b_tmp[idx] <= b_all[idx]:
            break
        else:
            b_tmp[idx] = b_tmp[idx] - b_all[idx]
