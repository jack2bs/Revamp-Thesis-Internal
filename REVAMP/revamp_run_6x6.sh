#!/bin/bash

echo "Revamp DSE started"
rm -rf *.log
python3 scripts/revamp_run_generator_6x6.py

DIR=$PWD/ARCHITECTURE_GENERATOR/generated_architectures
if [ "$(ls -A $DIR)" ]; then
	python3 scripts/revamp_run_mapper_6x6.py -x 6 -y 6&
	proc1=$!
	cp synthesis_scripts/synthesis_output/* synthesis_scripts/  
	# python3 scripts/revamp_run_synthesis_6x6.py&
    #     proc2=$!
	wait $proc1
	echo "Mapping Done"
	# wait $proc2
    #     echo "Synthesis Done"
	
	python3 scripts/revamp_run_analysis_6x6.py
else
	echo "Architecture generation not successful"
fi

