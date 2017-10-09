per:
	go build main_onlyPer.go parsing.go plotting_onlyPer.go
cpu_median:
	go build -o cpu_median main_cpu.go parsing.go plotting_median.go
gat_median:
	go build -o gat_median main_gat.go parsing.go plotting_median.go
cpu_box:
	go build -o cpu_box main_box.go parsing.go plotting_box.go
clean:
	rm -f main main_onlyPer gat_median cpu_median cpu_box *.tex *.eps *.pdf *.log *.aux
