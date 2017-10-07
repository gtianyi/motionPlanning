k="1"
j="0"
while (($k<26))
do
    while (($j<5))
    do
	cd ~/gopath/src/github.com/skiesel/moremotionplanning/experiments/data_KSLPI/anytime/300/1000/true/"$k"/KinematicCar/0.05/1/100/0.1/car2_planar_robot.dae/"$j"
	rm KEY=map
	((j=j+1))
	echo "$j"
    done
    ((j=0))
    ((k=k+1))
    echo "$k"
done
    
