#!/bin/bash

# First we need to compile from structuredslugs to slugsin

cd ../../etc/slugs/tools/StructuredSlugsParser
#python compiler.py ../../../../examples/multiRobot/two_robots_three_items/two_robots_three_items_instant_drop_one_goal.structuredslugs > ../../../../examples/multiRobot/two_robots_three_items/two_robots_three_items_instant_drop_one_goal.slugsin 
python compiler.py ../../../../examples/$1/$2.structuredslugs > ../../../../examples/$1/$2.slugsin 


# then we run only Realizability for SLUGS
cd ../../../../examples/$1

# third input to determine if we need to check realzability or the entire aut
if [ $3 == 'bdd' ] ; then
    echo 'Compiling bdd'
    time slugs --symbolicStrategy $2.slugsin  $2.bdd 
elif [ $3 == 'aut' ] ; then
    echo 'Compiling aut'
    time slugs $2.slugsin  $2.aut
else
    echo 'Only Realizability check'
    time slugs --onlyRealizability $2.slugsin 
fi
