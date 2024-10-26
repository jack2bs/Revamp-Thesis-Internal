  #!/usr/bin/env python
import sys
import os
import os.path
import shutil
import getopt
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import csv
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
    os.system('rm -rf analysis_reports')
    os.system('mkdir analysis_reports')
    outlog=open('analysis_reports/dse.rpt','w')
    print('\n---Analyzing the results---\n')
    outlog.write('\n---Analyzing the results---\n')

    
    with open(REVAMP_MAPPER_HOME+'/throughput.rpt', 'r') as infile:
        throughput = dict(csv.reader(infile))

    with open(REVAMP_SYN_HOME+'/power.rpt', 'r') as infile1:
        power = dict(csv.reader(infile1))

    with open(REVAMP_SYN_HOME+'/area.rpt', 'r') as infile2:
        area = dict(csv.reader(infile2))

    

    sorted_throughput,sorted_power,sorted_area=get_optimal(throughput,power,area)
    print(sorted_throughput)
    outlog.write('\nThroughput -->\n')
    outlog.write(str(sorted_throughput))
    print(sorted_power)
    outlog.write('\nPower Efficiency -->\n')
    outlog.write(str(sorted_power))
    print(sorted_area)
    outlog.write('\nArea Efficiency -->\n')
    outlog.write(str(sorted_area))

    compare_with_homo(throughput['hycube_original_4x4_torus'],throughput,power['hycube_original_4x4_torus'],power,area['hycube_original_4x4_torus'],area)

    outlog.write('\n---Pareto optimal designs---\n')
    print('\n---Pareto optimal designs---\n')
    outlog.write('\n---Optimal power efficiency---\n')
    print('\n---Optimal power efficiency---\n')
    max_pow=max(sorted_power,key=sorted_power.get)

    print(max_pow+'\n')
    
    print('Throughput impact:'+str((float(sorted_throughput['hycube_original_4x4_torus'])-float(sorted_throughput[max_pow]))*100/float(sorted_throughput['hycube_original_4x4_torus']))+'%\n')
    outlog.write(str(max_pow)+'\n')
    outlog.write('Throughput impact:'+str((float(sorted_throughput['hycube_original_4x4_torus'])-float(sorted_throughput[max_pow]))*100/float(sorted_throughput['hycube_original_4x4_torus']))+'%\n')

    print('\n---Optimal area efficiency---\n')
    outlog.write('\n---Optimal area efficiency---\n')
    max_area=max(sorted_area,key=sorted_area.get)

    print(max_area+'\n')
    outlog.write(str(max_area)+'\n')
    print('Throughput impact:'+str((float(sorted_throughput['hycube_original_4x4_torus'])-float(sorted_throughput[max_area]))*100/float(sorted_throughput['hycube_original_4x4_torus']))+'%\n')
    outlog.write('Throughput impact:'+str((float(sorted_throughput['hycube_original_4x4_torus'])-float(sorted_throughput[max_area]))*100/float(sorted_throughput['hycube_original_4x4_torus']))+'%\n')


    #compare_with_homo(throughput['hycube_original_4x4_torus'],throughput,power['hycube_original_4x4_torus'],power,area['hycube_original_4x4_torus'],area)
    
    plot_dse(throughput,power,area)
    
    end_time=time.time()
    outlog.write('\n Execution Time = '+str(end_time-start_time))
    print('\n Execution Time = '+str(end_time-start_time)+'\n')
    outlog.close()


def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass

def compare_with_homo(hm_thr,ht_thr,hm_pow,ht_pow,hm_ar,ht_ar):
    
    comp = open('analysis_reports/comparetoHomogeneous.rpt', 'w')
    #print(ht_thr)   
    for f in ht_thr:
        thr= float(float(ht_thr[f])/float(hm_thr))
        powx= float(float(ht_pow[f])/float(hm_pow))
        ar = float(float(ht_ar[f])/float(hm_ar))
        comp.write('Architecture-> \t'+f+'\n  Throughput-> \t'+str(thr)+'\n  Power-> \t'+str(powx)+'\n  Area-> '+str(ar)+'\n')
        print('Architecture-> \t'+f+'\n  Throughput-> \t'+str(thr)+'\n  Power-> \t'+str(powx)+'\n  Area-> '+str(ar)+'\n')
    comp.close()
 
def plot_dse(throughput,power,area):
    #print(len(throughput))
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    thr_n = np.empty(len(throughput))
    pow_n = np.empty(len(throughput))
    area_n = np.empty(len(throughput))
    i=0
    for archi in throughput:
        thr_n[i]=float(throughput[archi])
        pow_n[i]=float(power[archi])
        area_n[i]=float(area[archi])
        i=i+1
    #print(thr_n)
    zdata = thr_n
    xdata = pow_n
    ydata = area_n
    ax.scatter3D(xdata, ydata, zdata, cmap='viridis');
    ax.set_xlabel('Power(mW)')
    ax.set_ylabel('Area(um2)')
    ax.set_zlabel('Throughput(MOPS)')
    plt.savefig('analysis_reports/dse.png')


def get_optimal(throughput,output_power,output_area):
    
    power={}
    area={}
    
    for archi in throughput:
#        total=0
#        for app in heterogeneous[archi]:
#            total+=archi[app]
        #average throughput
#        throughput[archi] = total/len(heterogeneous[archi])

        name_t=archi.split('[0-9]*')
        
  #      for archip in output_power:
  #          name_p=archip.split('[0-9]*')
  #          if name_t==name_p:
                #print(name_p[0])
                #print(throughput[archi])
                #print(output_power[archip])
        power[archi]= float(float(throughput[archi])/float(output_power[archi]))
  #              print(power[archi])

  #      for archia in output_area:
  #          name_a=archia.split('[0-9]*')
  #          if name_t==name_a:
        area[archi]= float(float(throughput[archi])/float(output_area[archi]))

#    sorted_throughput = {}
#    sorted_throughput = sorted(throughput, key=throughput.get)
#
#    sorted_power = {}
#    sorted_power = sorted(power, key=power.get)
#
#    sorted_area = {}
#    sorted_area = sorted(area, key=area.get)


    return throughput,power,area

if __name__ == '__main__':
    
  main()
