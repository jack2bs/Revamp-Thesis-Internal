  #!/usr/bin/env python
import sys
import os
import os.path
import shutil
import getopt
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

    outlog=open('generator.log','w')
    print('\n---Generating heterogeneous architectures---\n')
    outlog.write('\n---Generating heterogeneous architectures---\n')
    os.chdir(REVAMP_ARCHI_GEN_HOME)
    os.system('rm -rf applications/ generated_architectures/* generated_architectures_RTL_config/*')
    os.system('mkdir generated_architectures/ generated_architectures_RTL_config/')
    os.system('cp -r '+REVAMP_APPLICATIONS+'/applications_4x4 '+ REVAMP_ARCHI_GEN_HOME+'/applications')
    print('src/build/generator -d applications -a '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_4x4_torus.json -c config.json')
    outlog.write('src/build/generator -d applications -a '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_4x4_torus.json -c config.json\n')
    os.system('src/build/generator -d applications -a '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_4x4_torus.json -c config.json >> /dev/null')
    print('\n---Generated heterogeneous architectures ---\n')
    outlog.write('\n---Generated heterogeneous architectures ---\n')
     
    end_time=time.time()
    outlog.write('\n Execution Time = '+str(end_time-start_time))
    print('\n Execution Time = '+str(end_time-start_time))
    outlog.close()


def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass

if __name__ == '__main__':
    
  main()
