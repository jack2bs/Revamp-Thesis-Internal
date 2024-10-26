  #!/usr/bin/env python
import sys
import os
import os.path
import shutil
import getopt
import subprocess

import time
start_time = time.time()
def main():
    
    REVAMP_HOME=os.getcwd()
    REVAMP_APPLICATIONS=REVAMP_HOME+'/APPLICATIONS'
    REVAMP_ARCHI_GEN_HOME=REVAMP_HOME+ '/ARCHITECTURE_GENERATOR'
    REVAMP_RTL_HOME=REVAMP_HOME+ '/HOMOGENEOUS_ARCHITECTURE/RTL'
    REVAMP_SYN_HOME=REVAMP_HOME+ '/synthesis_scripts'
    REVAMP_MAPPER_HOME=REVAMP_HOME+ '/HETEROGENEOUS_MAPPER' 
    REVAMP_HOMOGENEOUS_ARCHITECTURE=REVAMP_HOME+ '/HOMOGENEOUS_ARCHITECTURE'

    outlog=open('synthesis.log','w')

    print('\n---Running synthesis on generated architectures---\n') 
    outlog.write('\n---Running synthesis on generated architectures---\n')
    if len(os.listdir(REVAMP_ARCHI_GEN_HOME))==0:
        print('[ERROR]: No Architecture configurations generated!')
        outlog.write('[ERROR]: No Architecture configurations generated!\n')
        sys.exit(2)
    else:
        architectures = os.listdir(REVAMP_ARCHI_GEN_HOME+'/generated_architectures_RTL_config/')
    os.chdir(REVAMP_SYN_HOME)
    outlog.write("hycube_original_4x4_torus")
    filep = open('power.rpt', 'w')
    filea = open('area.rpt', 'w')

    file_object = open('define_setup.tcl', 'w')
    file_object.write('set DEFINES ""')
    file_object.close()
    os.system('rm -rf build* hycube*')
    os.system('make synth_4x4 >/dev/null')
    os.system('cp -r current/ hycube_original_4x4_torus')
    if(os.path.isfile('hycube_original_4x4_torus/reports/cgra.mapped.power.rpt')):
        os.system('cat hycube_original_4x4_torus/reports/cgra.mapped.power.rpt | grep -A 2 "combinational"|grep "Total"| cut -f4 -d\'W\'|sed \'s/^ *//g\'|cut -f1 -d\' \'>power.txt')
        with open('power.txt', 'r') as file:
            power = file.read().replace('\n', '')
            #print(power)
            outlog.write(power+' mW\n')
            if power!='':
                print(power)
                #outlog.write(power+' mW\n')
                filep.write('hycube_original_4x4_torus,'+power+'\n')
    else:
        print("[ERROR]: No power report generated! Synthesis Error!\n")
        outlog.write("[ERROR]: No power report generated! Synthesis Error!\n")
        sys.exit(2)

    if(os.path.isfile('hycube_original_4x4_torus/reports/cgra.mapped.area.rpt')):
        os.system('cat hycube_original_4x4_torus/reports/cgra.mapped.area.rpt|grep \'Total cell area:\'|cut -f2 -d\':\'|sed \'s/^ *//g\'>area.txt')
        with open('area.txt', 'r') as file:
            area = file.read().replace('\n', '')
            #print(area)
            outlog.write(area+' um2\n')
            if area!='':
                print(area)
                filea.write('hycube_original_4x4_torus,'+area+'\n')
    else:
        print("[ERROR]: No area report generated! Synthesis Error!\n")
        outlog.write("[ERROR]: No area report generated! Synthesis Error!\n")
        sys.exit(2)

#    output_area={}
#    output_power={}
    for arch in architectures:
        print(arch)
        outlog.write(arch)
        
        os.system('cp '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures_RTL_config/'+arch+' '+ REVAMP_RTL_HOME+'/hetero_config.v')
        defines="HETERO"
        if 'compute' in arch:
            defines=defines+"+HETERO_COMPUTE"
        if 'network' in arch:
            defines=defines+"+HETERO_NETWORK"
        if 'config' in arch:
            defines=defines+"+HETERO_CONFIG"

        #os.environ["DEFINES"]=defines
        file_object = open('define_setup.tcl', 'w')
        file_object.write('set DEFINES "'+defines+'"')
        file_object.close()

        os.system('make synth_4x4 >/dev/null')
        os.system('cp -r current/ ' +arch)
        if(os.path.isfile(arch+'/reports/cgra.mapped.power.rpt')):
            os.system('cat '+arch+'/reports/cgra.mapped.power.rpt | grep -A 2 "combinational"|grep "Total"| cut -f4 -d\'W\'|sed \'s/^ *//g\'|cut -f1 -d\' \'>power.txt')
            with open('power.txt', 'r') as file:
                power = file.read().replace('\n', '')
                #print(power)
                outlog.write(power+' mW\n')
                if power!='':
                    print(power)
                    x = arch.split(".")
                    filep.write(x[0]+','+power+'\n')
        else:
            print("[ERROR]: No power report generated! Synthesis Error!\n")
            outlog.write("[ERROR]: No power report generated! Synthesis Error!\n")
            sys.exit(2)	

        if(os.path.isfile(arch+'/reports/cgra.mapped.area.rpt')):
            os.system('cat '+arch+'/reports/cgra.mapped.area.rpt|grep \'Total cell area:\'|cut -f2 -d\':\'|sed \'s/^ *//g\'>area.txt')
            with open('area.txt', 'r') as file:
                 area = file.read().replace('\n', '')
                 #print(area)
                 outlog.write(area+' um2\n')
                 if area!='':
                    #output_area[arch]=float(area)
                    print(area)
                    x = arch.split(".")
                    filea.write(x[0]+','+area+'\n')
        else:
            print("[ERROR]: No area report generated! Synthesis Error!\n")
            outlog.write("[ERROR]: No area report generated! Synthesis Error!\n")
            sys.exit(2)	

    end_time=time.time()
    outlog.write('\n Execution Time = '+str(end_time-start_time))
    print('\n Execution Time = '+str(end_time-start_time)+'\n')
    outlog.close()
    filea.close()
    filep.close()

def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass

if __name__ == '__main__':
    
  main()
