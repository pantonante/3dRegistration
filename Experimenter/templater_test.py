#!/usr/bin/python3

from classes.Templater import *

def main():
	template = Templater('Templater Test')

	plot_1 = Plot('Test Plot (1)')
	plot_1.set_axis_label('sigma', 'MSE')
	plot_1.add_datapoints('p1', [1,2,3,4,5], [5,10,6,11,7])
	plot_1.add_datapoints('p2', [5,10,15,20,25], [11,12,13,14,15])
	template.add_plot(plot_1)
	
	plot_2 = Plot('Test Plot (2)')
	plot_1.set_axis_label('delta', 'f(x)')
	plot_2.add_datapoints('triangle', [1,2,3,4,5], [0.1,0.2,0.3,0.2,0.1])
	template.add_plot(plot_2)

	plot_3 = ScatterPlot3('Scatter plot')
	plot_3.set_axis_label('sigma', 'param', 'MSE')
	plot_3.add_datapoints('Scatter 3d', [0,6,5,-1,3,6,2,0,3], [0,1,5,5,1,3,6,3,3.75], [1,16,1.3,0,0,0.4,1.1,0.9,0.5])
	template.add_plot(plot_3)

	plot_4= BoxPlot('Box Plot')
	plot_4.add_datapoints({
		'Step 1': [0.3915579796014891, 0.36477900638056404, 0.6072548759908706, 0.9570577095660634, 0.20294846993639903, 0.5380495405677727, 0.4303427003079918, 0.3214100602810884, 0.44033572748453964, 0.7009581444403025], 
		'Step 2': [0.9440918437895138, 0.8152791973137701, 1.6308979042607759, 0.7201062053846804, 0.4093153519750409, 0.378927598942539, 1.7524235550550498, 1.0590062646351512, 0.964024748819738, 1.4522703260845724], 
		'Step 3': [0.08078693780914833, 0.34282747157072474, 0.28326899111770865, 0.07079067171526338, 0.053445854836727105, 0.06742411658502723, 0.27104285620634416, 0.8206746854582878, 0.19637789716752194, 0.2018530215834835]
		})
	plot_4.set_axis_label('Seconds')
	template.add_plot(plot_4)

	template.show()
	template.render('test.tex', 'tex')
	template.render('test.html', 'html')

##############################################################

if __name__ == "__main__":
	main()