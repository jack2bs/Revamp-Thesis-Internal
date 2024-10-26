#!/bin/bash

echo "Revamp DSE started"
rm -rf *.log
python3 scripts/revamp_run_generator_4x4.py

DIR=$PWD/ARCHITECTURE_GENERATOR/generated_architectures
if [ "$(ls -A $DIR)" ]; then
	python3 scripts/revamp_run_mapper_4x4.py -x 4 -y 4&
	proc1=$!
	cp synthesis_scripts/synthesis_output/* synthesis_scripts/  
#	python3 scripts/revamp_run_synthesis_4x4.py&
#        proc2=$!
	wait $proc1
	echo "Mapping Done"
#	wait $proc2
#        echo "Synthesis Done"
	
	python3 scripts/revamp_run_analysis_4x4.py
else
	echo "Architecture generation not successful"
fi

