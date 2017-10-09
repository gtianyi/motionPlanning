package main

import (
	"fmt"
	"math"
	//"os"
	"sort"
	"strconv"
	"strings"

	"github.com/skiesel/plot"
	"github.com/skiesel/plot/plotter"
	//	"github.com/skiesel/plot/plotutil"
	"github.com/skiesel/plot/vg"

	//	"github.com/skiesel/expsys/plots"
	"github.com/skiesel/expsys/rdb"
)

type ParallelSlices struct {
	Values []float64
	Labels []string
}

func (a ParallelSlices) Len() int {
	return len(a.Values)
}

func (a ParallelSlices) Less(i, j int) bool {
	return a.Values[i] < a.Values[j]
}

func (a ParallelSlices) Swap(i, j int) {
	a.Values[i], a.Values[j] = a.Values[j], a.Values[i]
	a.Labels[i], a.Labels[j] = a.Labels[j], a.Labels[i]
}

//func makeCompactBoxPlot(title, yLabel, key, format string, width, height float64, experiment []*rdb.Dataset, log10, tryStacked bool,_ymax float64,mmap string) {
func makeCompactBoxPlot(title, yLabel, key, format string, experiments map[string][]*rdb.Dataset, width, height float64) {

	for mapType, experiment := range experiments {

		p, err := plot.New()
		if err != nil {
			panic(err)
		}

		p.Title.Text = title + "_" + mapType
		p.Y.Label.Text = yLabel

		dsSizes := map[string]float64{}

		//yMax := 0.

		slices := ParallelSlices{
			Values: []float64{},
			Labels: []string{},
			//Extra:  []float64{},
		}
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
				//kpieceAlgSet = ds
				//pprmAlgSet = ds
			} else if ds.GetName() == "P-PRM" {
				pprmAlgSet = ds
			} else if ds.GetName() == "KPIECE" {
				kpieceAlgSet = ds
				//rrtAlgSet = ds
			} else if ds.GetName() == "RRT" {
				rrtAlgSet = ds
				//beastAlgSet = ds
			}
			dsSizes[ds.GetName()] = float64(ds.GetSize())
			slices.Labels = append(slices.Labels, ds.GetName())
			mean := 0.0
			for _, val := range ds.GetDatasetFloatValues(key) {
				mean = mean + val
			}
			slices.Values = append(slices.Values, mean/float64(ds.GetSize()))
		}
		fmt.Println(mapType)
		//		fmt.Println(beastAlgSet.GetName())
		//		fmt.Println(pprmAlgSet.GetName())
		//		fmt.Println(kpieceAlgSet.GetName())
		fmt.Println(beastAlgSet.GetSize())
		fmt.Println(pprmAlgSet.GetSize())
		fmt.Println(kpieceAlgSet.GetSize())
		fmt.Println(rrtAlgSet.GetSize())
		if pprmAlgSet.GetSize() == 0 {
			continue
		}

		sort.Sort(slices)

		w := vg.Points(20)
		i := 0.
		plotters := []plot.Plotter{}
		labels := []string{}
		for _, algorithmName := range slices.Labels {
			var algorithm *rdb.Dataset
			for _, ds := range experiment {
				if ds.GetName() == algorithmName {
					algorithm = ds
					break
				}
			}

			if algorithm == nil {
				panic("failed to find algorithm in dataset")
			}

			data := plotter.Values{}

			for _, val := range algorithm.GetDatasetFloatValues(key) {
				data = append(data, val)
			}

			var box plot.Plotter

			box, err = plotter.NewBoxPlotWithConfidenceIntervals(w, i, data)

			if err != nil {
				fmt.Println(len(data))
				fmt.Println(algorithm.GetName())
				panic(err)
			}

			plotters = append(plotters, box)
			labels = append(labels, strings.Replace(algorithm.GetName(), " ", "\n", -1))
			i++
		}

		p.Add(plotters...)
		p.NominalX(labels...)
		p.Y.Max = 30

		filename := strings.Replace(p.Title.Text, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + "test" + format

		fmt.Println(filename)

		if err := p.Save(vg.Length(width)*vg.Points(15*float64(len(plotters))), vg.Length(height)*vg.Inch, filename); err != nil {
			panic(err)
		}
	}

	return
}

func makeCompactBoxPlotGAT(title, yLabel, key string, experiments map[string][]*rdb.Dataset, rdata map[string]map[string]string) map[string]map[string]string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel

	//	slices := ParallelSlices{
	//		Values: []float64{},
	//		Labels: []string{},
	//	}

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
				//kpieceAlgSet = ds
				//pprmAlgSet = ds
			} else if ds.GetName() == "P-PRM" {
				pprmAlgSet = ds
			} else if ds.GetName() == "KPIECE" {
				kpieceAlgSet = ds
				//rrtAlgSet = ds
			} else if ds.GetName() == "RRT" {
				rrtAlgSet = ds
				//beastAlgSet = ds
			}
		}
		fmt.Println(mapType)
		//		fmt.Println(beastAlgSet.GetName())
		//		fmt.Println(pprmAlgSet.GetName())
		//		fmt.Println(kpieceAlgSet.GetName())
		fmt.Println(beastAlgSet.GetSize())
		fmt.Println(pprmAlgSet.GetSize())
		fmt.Println(kpieceAlgSet.GetSize())
		fmt.Println(rrtAlgSet.GetSize())
		if pprmAlgSet.GetSize() == 0 {
			continue
		}

		precomputationTime_beast := beastAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_beast := beastAlgSet.GetDatasetFloatValues(key)
		solutiontime_beast := beastAlgSet.GetDatasetFloatValues("Solution Length")
		//slncost_beast := beastAlgSet.GetDatasetFloatValues("solution cost")
		// dsValues := beastAlgSet.GetColumnValuesWithKeys("solution", []string{"inst", "seed", "Precomputation Time"}, "solution time", "solution cost")
		//		fmt.Println  // this may be can get solution cost, have not test yet.
		//		fmt.Println(len(cuptime_beast))
		//		fmt.Println(len(slncost_beast))
		//		fmt.Println(cuptime_beast[0])
		//		fmt.Println(slncost_beast[0])
		precomputationTime_pprm := pprmAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_pprm := pprmAlgSet.GetDatasetFloatValues(key)
		solutiontime_pprm := pprmAlgSet.GetDatasetFloatValues("Solution Length")
		precomputationTime_kpiece := kpieceAlgSet.GetDatasetFloatValues("Precomputation Time")
		cuptime_kpiece := kpieceAlgSet.GetDatasetFloatValues(key)
		solutiontime_kpiece := kpieceAlgSet.GetDatasetFloatValues("Solution Length")
		cuptime_rrt := rrtAlgSet.GetDatasetFloatValues(key)
		solutiontime_rrt := rrtAlgSet.GetDatasetFloatValues("Solution Length")

		var fac_pprm []float64
		var fac_kpiece []float64
		var fac_rrt []float64

		for i := 0; i < pprmAlgSet.GetSize(); i++ {
			if cuptime_pprm[i] >= 300 {
				cuptime_pprm[i] = 100000
			}
			if cuptime_kpiece[i] >= 300 {
				cuptime_kpiece[i] = 100000
			}
			if cuptime_rrt[i] >= 300 {
				cuptime_rrt[i] = 100000
			}
			fac_pprm = append(fac_pprm, (cuptime_pprm[i]+precomputationTime_pprm[i]+solutiontime_pprm[i])/(cuptime_beast[i]+precomputationTime_beast[i]+solutiontime_beast[i]))
			fac_kpiece = append(fac_kpiece, (cuptime_kpiece[i]+precomputationTime_kpiece[i]+solutiontime_kpiece[i])/(cuptime_beast[i]+precomputationTime_beast[i]+solutiontime_beast[i]))
			fac_rrt = append(fac_rrt, (cuptime_rrt[i]+solutiontime_rrt[i])/(cuptime_beast[i]+precomputationTime_beast[i]+solutiontime_beast[i]))
		}

		//sort
		sort.Float64s(fac_pprm)
		sort.Float64s(fac_kpiece)
		sort.Float64s(fac_rrt)

		//get median position
		n := float64(pprmAlgSet.GetSize())
		j := int(math.Ceil(n*0.5 - 1.96*math.Sqrt(n*0.5*0.5)))
		k := int(math.Ceil(n*0.5 + 1.96*math.Sqrt(n*0.5*0.5)))

		lci_pprm := fac_pprm[j]
		uci_pprm := fac_pprm[k]
		lci_kpiece := fac_kpiece[j]
		uci_kpiece := fac_kpiece[k]
		lci_rrt := fac_rrt[j]
		uci_rrt := fac_rrt[k]

		pprm_po := 1
		kpiece_po := 1
		rrt_po := 1
		if lci_pprm > 10 {
			pprm_po = 0
		}
		if lci_kpiece > 10 {
			kpiece_po = 0
		}
		if lci_rrt > 10 {
			rrt_po = 0
		}
		lci_pprm_str := strconv.FormatFloat(lci_pprm, 'f', pprm_po, 64)
		if lci_pprm > 200 {
			lci_pprm_str = "$\\infty$"
		}
		uci_pprm_str := strconv.FormatFloat(uci_pprm, 'f', pprm_po, 64)
		if uci_pprm > 200 {
			uci_pprm_str = "$\\infty$"
		}
		lci_kpiece_str := strconv.FormatFloat(lci_kpiece, 'f', kpiece_po, 64)
		if lci_kpiece > 200 {
			lci_kpiece_str = "$\\infty$"
		}
		uci_kpiece_str := strconv.FormatFloat(uci_kpiece, 'f', kpiece_po, 64)
		if uci_kpiece > 200 {
			uci_kpiece_str = "$\\infty$"
		}
		lci_rrt_str := strconv.FormatFloat(lci_rrt, 'f', rrt_po, 64)
		if lci_rrt > 200 {
			lci_rrt_str = "$\\infty$"
		}
		uci_rrt_str := strconv.FormatFloat(uci_rrt, 'f', rrt_po, 64)
		if uci_rrt > 200 {
			uci_rrt_str = "$\\infty$"
		}

		rdata[mapType]["P-PRM"] = rdata[mapType]["P-PRM"] + "&" +
			lci_pprm_str + "--" + uci_pprm_str
		rdata[mapType]["KPIECE"] = rdata[mapType]["KPIECE"] + "&" +
			lci_kpiece_str + "--" + uci_kpiece_str
		rdata[mapType]["RRT"] = rdata[mapType]["RRT"] + "&" +
			lci_rrt_str + "--" + uci_rrt_str

	}

	return rdata
}

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
		//		fmt.Println(beastAlgSet.GetName())
		//		fmt.Println(pprmAlgSet.GetName())
		//		fmt.Println(kpieceAlgSet.GetName())
		fmt.Println(beastAlgSet.GetSize())
		fmt.Println(pprmAlgSet.GetSize())
		fmt.Println(kpieceAlgSet.GetSize())
		fmt.Println(rrtAlgSet.GetSize())

		//precomputationTime_beast := beastAlgSet.GetDatasetFloatValues("Precomputation Time")
		//cuptime_beast := beastAlgSet.GetDatasetFloatValues(key)
		p_sol := pprmAlgSet.GetDatasetStringValues("Solved")
		k_sol := kpieceAlgSet.GetDatasetStringValues("Solved")
		r_sol := rrtAlgSet.GetDatasetStringValues("Solved")

		p_sol_per := 0.0
		k_sol_per := 0.0
		r_sol_per := 0.0
		for i := 0; i < pprmAlgSet.GetSize(); i++ {
			if p_sol[i] == "true" {
				p_sol_per++
			}
			if k_sol[i] == "true" {
				k_sol_per++
			}
			if r_sol[i] == "true" {
				r_sol_per++
			}
		}
		p_sol_per = p_sol_per / float64(pprmAlgSet.GetSize())
		k_sol_per = k_sol_per / float64(pprmAlgSet.GetSize())
		r_sol_per = r_sol_per / float64(pprmAlgSet.GetSize())

		p_sol_per_str := strconv.FormatFloat(p_sol_per, 'f', 3, 64)
		k_sol_per_str := strconv.FormatFloat(k_sol_per, 'f', 3, 64)
		r_sol_per_str := strconv.FormatFloat(r_sol_per, 'f', 3, 64)

		rdata[mapType]["P-PRM"] = rdata[mapType]["P-PRM"] + "&" +
			p_sol_per_str
		rdata[mapType]["KPIECE"] = rdata[mapType]["KPIECE"] + "&" +
			k_sol_per_str
		rdata[mapType]["RRT"] = rdata[mapType]["RRT"] + "&" +
			r_sol_per_str
	}

	return rdata
}
