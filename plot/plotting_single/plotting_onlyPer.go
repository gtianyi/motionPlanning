package main

import (
	"fmt"
	//"math"
	//	"os"
	//	"sort"
	"strconv"
	//	"strings"

	"github.com/skiesel/plot"
	//	"github.com/skiesel/plot/plotter"
	//	"github.com/skiesel/plot/plotutil"
	//	"github.com/skiesel/plot/vg"

	//	"github.com/skiesel/expsys/plots"
	"github.com/skiesel/expsys/rdb"
)

func makeCompactBoxPlotSolvePerc(title, yLabel, key string, experiments map[string][]*rdb.Dataset, rdata map[string]map[string]string) map[string]map[string]string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel

	for mapType, experiment := range experiments {
		//fmt.Fprintf(latex, "%s&",strings.Replace(mapType,".dae","",-1))
		var beastAlgSet *rdb.Dataset
		var pprmAlgSet *rdb.Dataset
		var kpieceAlgSet *rdb.Dataset
		var rrtAlgSet *rdb.Dataset

		for _, ds := range experiment {
			if !ds.TestDataset(func(val string) bool { return val == "true" }, "Solved") {
				fmt.Println("no!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			}
			if ds.GetName() == "BEAST" {
				beastAlgSet = ds
			} else if ds.GetName() == "P-PRM" {
				pprmAlgSet = ds
			} else if ds.GetName() == "KPIECE" {
				kpieceAlgSet = ds
			} else if ds.GetName() == "RRT" {
				rrtAlgSet = ds
			}
		}
		fmt.Println(mapType)
		fmt.Println(beastAlgSet.GetSize())
		fmt.Println(pprmAlgSet.GetSize())
		fmt.Println(kpieceAlgSet.GetSize())
		fmt.Println(rrtAlgSet.GetSize())

		//precomputationTime_beast := beastAlgSet.GetDatasetFloatValues("Precomputation Time")
		//cuptime_beast := beastAlgSet.GetDatasetFloatValues(key)
		p_sol := pprmAlgSet.GetDatasetStringValues("Solved")
		k_sol := kpieceAlgSet.GetDatasetStringValues("Solved")
		r_sol := rrtAlgSet.GetDatasetStringValues("Solved")
		b_sol := beastAlgSet.GetDatasetStringValues("Solved")

		cuptime_beast := beastAlgSet.GetDatasetFloatValues(key)
		pt_beast := beastAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_pprm := pprmAlgSet.GetDatasetFloatValues(key)
		pt_pprm := pprmAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_kpiece := kpieceAlgSet.GetDatasetFloatValues(key)
		pt_kpiece := kpieceAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_rrt := rrtAlgSet.GetDatasetFloatValues(key)

		threshold := 300.0 //we use this to make sure it solve the problem, in case OMPL is wrong.

		p_sol_per := 0.0
		k_sol_per := 0.0
		r_sol_per := 0.0
		b_sol_per := 0.0
		for i := 0; i < pprmAlgSet.GetSize(); i++ {
			if p_sol[i] == "true" && cuptime_pprm[i]+pt_pprm[i] <= threshold {
				p_sol_per++
			}
			if k_sol[i] == "true" && cuptime_kpiece[i]+pt_kpiece[i] <= threshold {
				k_sol_per++
			}
			if r_sol[i] == "true" && cuptime_rrt[i] <= threshold {
				r_sol_per++
			}
			if b_sol[i] == "true" && cuptime_beast[i]+pt_beast[i] <= threshold {
				b_sol_per++
			}
		}
		p_sol_per = p_sol_per / float64(pprmAlgSet.GetSize())
		k_sol_per = k_sol_per / float64(pprmAlgSet.GetSize())
		r_sol_per = r_sol_per / float64(pprmAlgSet.GetSize())
		b_sol_per = b_sol_per / float64(pprmAlgSet.GetSize())

		p_sol_per_str := strconv.FormatFloat(p_sol_per, 'f', 3, 64)
		k_sol_per_str := strconv.FormatFloat(k_sol_per, 'f', 3, 64)
		r_sol_per_str := strconv.FormatFloat(r_sol_per, 'f', 3, 64)
		b_sol_per_str := strconv.FormatFloat(b_sol_per, 'f', 3, 64)

		rdata[mapType]["P-PRM"] = rdata[mapType]["P-PRM"] + "&" +
			p_sol_per_str
		rdata[mapType]["KPIECE"] = rdata[mapType]["KPIECE"] + "&" +
			k_sol_per_str
		rdata[mapType]["RRT"] = rdata[mapType]["RRT"] + "&" +
			r_sol_per_str
		rdata[mapType]["BEAST"] = rdata[mapType]["BEAST"] + "&" +
			b_sol_per_str
	}

	return rdata
}
