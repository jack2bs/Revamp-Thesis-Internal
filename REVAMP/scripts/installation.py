#!/usr/bin/env python
import sys
import os
import os.path
import shutil
import time
def main():
    start_time=time.time()
    REVAMP_HOME=os.getcwd()
    ARCHI_GEN_HOME=REVAMP_HOME+ '/ARCHITECTURE_GENERATOR'
    MAPPER_HOME=REVAMP_HOME+ '/HETEROGENEOUS_MAPPER' 

    print("\n---Building Architecture Generator---\n")
    os.system('rm -rf '+ARCHI_GEN_HOME+'/src/build')
    my_mkdir(ARCHI_GEN_HOME+'/src/build')
    os.chdir(ARCHI_GEN_HOME+'/src/build')
    os.system('cmake ..')
    os.system('make all -j6')

    print("\n---Building Heterogeneous Mapper---\n")
    os.system('rm -rf '+MAPPER_HOME+'/src/build')
    my_mkdir(MAPPER_HOME+'/src/build')
    os.chdir(MAPPER_HOME+'/src/build')
    os.system('cmake ..')
    os.system('make all -j6')

    os.chdir(REVAMP_HOME)
    print('Installation Time = '+str(time.time()-start_time))


def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass

if __name__ == '__main__':
  main()
