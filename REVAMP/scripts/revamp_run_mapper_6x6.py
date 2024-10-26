  #!/usr/bin/env python
import sys
import os
import os.path
import shutil
import getopt

import time
start_time = time.time()
def main():
    
    try:
      opts, args = getopt.getopt(sys.argv[1:],"hx:y:",["X=","Y="])
    except getopt.GetoptError:
      print ('Provide CGRA size(X,Y)')
      sys.exit(2)

    for opt, arg in opts:
      if opt == '-h':
         print('Provide CGRA size(X,Y)')
         sys.exit()
      elif opt in ("-x", "--X"):
         x = arg
      elif opt in ("-y", "--Y"):
         y = arg

    REVAMP_HOME=os.getcwd()
    REVAMP_APPLICATIONS=REVAMP_HOME+'/APPLICATIONS'
    REVAMP_ARCHI_GEN_HOME=REVAMP_HOME+ '/ARCHITECTURE_GENERATOR'
    REVAMP_RTL_HOME=REVAMP_HOME+ '/HOMOGENEOUS_ARCHITECTURE/RTL'
    REVAMP_SYN_HOME=REVAMP_HOME+ '/synthesis_scripts'
    REVAMP_MAPPER_HOME=REVAMP_HOME+ '/HETEROGENEOUS_MAPPER' 
    REVAMP_HOMOGENEOUS_ARCHITECTURE=REVAMP_HOME+ '/HOMOGENEOUS_ARCHITECTURE'

    outlog=open('mapper.log','w')

    print('\n---Running applications on generated architectures---\n')
    outlog.write('\n---Running applications on generated architectures---\n')
    os.chdir(REVAMP_MAPPER_HOME)
    architectures = os.listdir(REVAMP_ARCHI_GEN_HOME+'/generated_architectures/')
    os.system('rm -rf applications/')
    os.system('cp -r '+REVAMP_APPLICATIONS+'/applications_6x6 '+ REVAMP_MAPPER_HOME+'/applications')

    applications = os.listdir(REVAMP_APPLICATIONS+'/applications_6x6')
    filex = open('throughput.rpt', 'w')

    #homogeneous ={}
    node_count={}
    freq=100 #in MHz
    #filex.write('Homogeneous\n')
    maxII=0
    totalhm=0
    for app in applications:
       	os.system('cat applications/'+app+'|grep "DFG count"|cut -d \'"\' -f 2 > count')
        with open('count', 'r') as file:
            count = file.read().replace('\n', '')
        node_count[app]=int(count)
        print('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_6x6_torus.json -x ' +str(x)+ ' -y '+str(y)+' -i 4')
        outlog.write('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_6x6_torus.json -x ' +str(x)+ ' -y '+str(y)+' -i 4\n')
        os.system('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_HOMOGENEOUS_ARCHITECTURE + '/hycube_original_6x6_torus.json -x ' +str(x)+ ' -y '+str(y)+' -i 4  |grep "MAPPING_RESULT"| cut -d "=" -f 2|grep -o \'[0-9]*\'>homo_II')
        with open('homo_II', 'r') as file:
            ii = file.read().replace('\n', '')
        
        print(app+'  -->  '+ii)
        outlog.write("Homogeneous baseline-> "+app+'  -->  II='+ii+'\n')
        #filex.write('hycube_original_6x6_torus.json'+','+'app+'  II='+ii)
        if int(ii)>int(maxII):
            maxII=ii
        totalhm+=float(freq*node_count[app]/int(ii))
    filex.write('hycube_original_6x6_torus'+','+str(totalhm/len(applications))+'\n')
    #homogeneous['hycube_original_6x6_torus.json']=totalhm/len(applications)

    #heterogeneous={}
    #filex.write('Heterogeneous\n')
    for f in architectures:
        compute=0
        network=0
        config=0


        if 'compute' in f:
            compute=1
        if 'network' in f:
            network=1
        if 'config' in f:
            config=1
        hetero_app={}
        totalht=0
        all_run=1
        for app in applications:
        
            if network and config:
                print('src/build/heterogeneous_compiler_nc -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/'+ f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII)
                outlog.write('src/build/heterogeneous_compiler_nc -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/'+ f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+'\n')
                os.system('src/build/heterogeneous_compiler_nc -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/'+ f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+' |grep "MAPPING_RESULT"| cut -d "=" -f 2|grep -o \'[0-9]*\'>hetero_II')
            elif network:
                print('src/build/heterogeneous_compiler_n -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII)
                outlog.write('src/build/heterogeneous_compiler_n -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+'\n')
                os.system('src/build/heterogeneous_compiler_n -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+' |grep "MAPPING_RESULT"| cut -d "=" -f 2|grep -o \'[0-9]*\'>hetero_II')
            elif config:
                print('src/build/heterogeneous_compiler_c -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII)
                outlog.write('src/build/heterogeneous_compiler_c -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+'\n')
                os.system('src/build/heterogeneous_compiler_c -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+' |grep "MAPPING_RESULT"| cut -d "=" -f 2|grep -o \'[0-9]*\'>hetero_II')
            else:
                print('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII )
                outlog.write('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+'\n' )
                os.system('src/build/homogeneous_compiler -d applications/'+app+ ' -j '+REVAMP_ARCHI_GEN_HOME+'/generated_architectures/' + f + ' -x ' +str(x)+ ' -y '+str(y)+' -i 4 -q '+ maxII+' |grep "MAPPING_RESULT"| cut -d "=" -f 2|grep -o \'[0-9]*\'>hetero_II')
            with open('hetero_II', 'r') as file:
                iix = file.read().replace('\n', '')

            print(f+ '---> '+app+'  -->  '+iix)
            outlog.write(f+ '---> '+app+'  -->  II='+iix+'\n')
            if iix !="0":
                totalht+=float(freq*node_count[app]/int(iix))
            else:
                all_run=0
                break;
            #filex.write(f+ '---> '+app+'  --> II='+iix)
            #totalht+=float(freq*node_count[app]/int(ii))
        if all_run:
            x = f.split(".")
            filex.write(x[0]+','+str(totalht/len(applications))+'\n')

        #heterogeneous[f]=hetero_app

    end_time=time.time()
    outlog.write('\n Execution Time = '+str(end_time-start_time))
    print('\n Execution Time = '+str(end_time-start_time)+'\n')
    filex.close()
    outlog.close()


def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass
 

if __name__ == '__main__':
    
    main()
