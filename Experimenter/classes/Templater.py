from jinja2 import Environment, FileSystemLoader
import numpy as np
import os

##################################################

class Plot(object):
  def __init__(self, title):
    self.plot = {
      'type': 'lineplot',
      'title': title ,
      'x-axis': 'x-axis',
      'y-axis': 'y-axis',
      'data': []
    }

  def set_axis_label(self, x_axis, y_axis):
    self.plot['x-axis'] = x_axis
    self.plot['y-axis'] = y_axis

  def add_datapoints(self, name, X, Y):
    self.plot['data'].append({
      'legend': name,
      'X': X,
      'Y': Y
      })

  def show(self):
    print(self.plot)

  def toDictionary(self):
    return self.plot

##################################################

class ScatterPlot3(Plot):

    def __init__(self, title):
        self.plot = {
            'type': 'scatter3',
            'title': title ,
            'x-axis': 'x-axis',
            'y-axis': 'y-axis',
            'z-axis': 'z-axis',
            'data': []
        }

    def set_axis_label(self, x_axis, y_axis, z_axis):
      self.plot['x-axis'] = x_axis
      self.plot['y-axis'] = y_axis
      self.plot['z-axis'] = z_axis

    def add_datapoints(self, name, X, Y, Z):
      self.plot['data'].append({
                        'legend': name,
                        'X': X,
                        'Y': Y,
                        'Z': Z
                    })

##################################################

class BoxPlot(Plot):

    def __init__(self, title):
        self.plot = {
            'type': 'boxplot',
            'title': title ,
            'y-axis': '',
            'data': {},
            'precomputed': {}
        }

    def add_datapoints(self, data):
      self.plot['data'] = data
      self.plot['precomputed'] = self.__precompute(data)

    def set_axis_label(self, y_axis):
      self.plot['y-axis'] = y_axis

    def __precompute(self, data):
      c_data = data # computed_data
      for key,val in c_data.items():
        raw = np.array(val)
        c_data[key] = [np.min(raw), np.percentile(raw, 25), np.percentile(raw, 50), np.mean(raw), np.percentile(raw, 75), np.max(raw)]
      return c_data

##################################################

class Templater(object):

    PATH = os.path.dirname(os.path.abspath(__file__))
    TEMPLATE_ENVIRONMENT = Environment(
      autoescape=False,
      loader=FileSystemLoader(os.path.join(PATH, 'templates')),
      trim_blocks=True)

    def __init__(self, docname):
        self.context = {
            'name': docname,
            'plots': []
        }

    def add_plot(self, plot):
      if not isinstance(plot, Plot):
        raise TypeError()
      self.context['plots'].append(plot.toDictionary())

    def show(self):
      print(self.context)

    def toDictionary(self):
      return self.contex

    def render(self, filename, template_type):
      if not self.context['plots']:
        return False

      if ('tex' or 'latex') in template_type:
        template_filename = 'latex.tex'
      elif ('html' or 'HTML') in template_type:
        template_filename = 'html5.html'
      else:
        raise ValueError()

      with open(filename, 'w') as f:
          render = self.TEMPLATE_ENVIRONMENT.get_template(template_filename).render(self.context)
          f.write(render)

      return True

##################################################
