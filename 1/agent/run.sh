#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="mapping.out"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        python3 c1.py -h "$host" -p "$pos" --robname "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 c2.py -h "$host" -p "$pos" --robname "$robname" # assuming -f is the option for the map
        mv a.out "$outfile"
        ;;
    3)
        # how to call agent for challenge 3
        python3 c3.py -h "$host" -p "$pos" --robname "$robname" # assuming -f is the option for the path
        mv b.out "$outfile"
        ;;
esac

