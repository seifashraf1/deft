#!/bin/bash

main_paths_arr=()
function search_for_Main(){
current_pwd=$1
filename=$2


for di in $current_pwd/*
	do
		#echo $di
		if [[ -f $di && $di =~ .*main.cpp ]];then
			main_paths_arr+=($di)
		fi
		if [[ -d $di ]];then
			#echo $di		
			search_for_Main $di
		fi
	done

}
function testing_main_files(){
for value in "${main_paths_arr[@]}"
do
     #echo $value 
     #echo -n $value 
     #echo $main| grep ".*\n"
     command="clang++-9 -g -O3 $value `llvm-config-9 --cxxflags --ldflags --libs` -std=c++17 -o try"
     
     $command 2> $(dirname $value)/err.txt
     
     if [[ $(find $(dirname $value) -empty -name err.txt) =~ "err.txt" ]]
     then
      echo "succeed -> $value"
     else
      echo "failed -> $value"
     fi
     
     
done
#command=clang++-9 -g -O3 main.cpp `llvm-config-9 --cxxflags --ldflags --libs` -std=c++17 -o try
}

current_pwd="$(pwd)/tools"
filename="main.cpp"
search_for_Main $current_pwd $filename

testing_main_files 

