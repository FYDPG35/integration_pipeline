from __future__ import division
import sys
import numpy
import os
import math
import urllib2
import cv2
import numpy as np
from geopy.geocoders import Nominatim

ZOOM = 18
HEIGHT = 512
DISTANCE_1M = 4 # Distance in pixels for 50 meters

MANUAL_MODE = 0
AUTONOMOUS_MODE = 1

WINDOW_NAME = "Display Window"

class Vertex:
	x = 0.0
	y = 0.0
	id = -1
	neighbors = []


class Path:

	def __init__(self, resolution = 1, radius = 10*DISTANCE_1M):
		self.rows = 0
		self.cols = 0
		self.resolution = resolution
		self.radius = radius
		self.vertices = []


	def add_corners(self, corner_1, corner_2, corner_3, corner_4):
		# Add the GPS -> meters conversion here
		""" These corners should be in meters"""
		self.corners = [corner_1, corner_2, corner_3, corner_4]
		print self.corners
		self.min_x = min(self.corners,key=lambda item:item[0])[0]
		self.max_x = max(self.corners,key=lambda item:item[0])[0]
		self.min_y = min(self.corners,key=lambda item:item[1])[1]
		self.max_y = max(self.corners,key=lambda item:item[1])[1]
		self.range_x = self.max_x - self.min_x
		self.range_y = self.max_y - self.min_y
		# self.rows = math.ceil(self.range_x/self.resolution)
		# self.cols = math.ceil(self.range_y/self.resolution)
		# self.resolution_x = self.range_x/self.rows
		# self.resolution_y = self.range_y/self.cols
		# print "X Range: %f" % self.range_x
		# print "Y Range: %f" % self.range_y


	def add_vertices(self):
		self.rows = math.ceil(self.range_x/(2*self.radius))
		self.cols = math.ceil(self.range_y/(2*self.radius))
		print "Vertical: %f" % self.rows
		print "Horizontal: %f" % self.cols

		id = 0
		for i in range(int(self.rows)):
			for j in range(int(self.cols)):
				vertex = Vertex()
				vertex.x = self.min_x + (1 + 2*i)*self.range_x/(2*self.rows)
				if i%2 == 0:
					vertex.y = self.min_y + (1 + 2*j)*self.range_y/(2*self.cols)
				else:
					vertex.y = self.min_y + (2*self.cols - 1 - 2*j)*self.range_y/(2*self.cols)

				print "Node: (%f, %f)" % (vertex.x, vertex.y)
				vertex.id = id
				for node in self.vertices:
					distance = math.sqrt(pow(vertex.x - node.x, 2) + pow(vertex.y - node.y, 2))
					vertex.neighbors.append((node, distance))
				self.vertices.append(vertex)
				id = id + 1

		print "Number of Vertices: %d " % len(self.vertices)

	def plan_path(self):
		self.add_vertices()


""" MAPS Manager """
class MapManager(object):

	BASE_URL = "http://maps.googleapis.com/maps/api/staticmap"

	def __init__(self, map_height, zoom, lat, lon, mode, rectangle_size=100):
		self.done = False
		self.map_height = map_height
		self.zoom = zoom
		self.static_map = self.make_map_request(lat, lon)
		self.center_lat = lat
		self.center_lon = lon
		# Add the rectangle corners
		self.plotted_points = []
		if mode:
			self.corners = [(256 - (rectangle_size/2)*DISTANCE_1M, 256 - (rectangle_size/2)*DISTANCE_1M),
							(256 - (rectangle_size/2)*DISTANCE_1M, 256 + (rectangle_size/2)*DISTANCE_1M),
							(256 + (rectangle_size/2)*DISTANCE_1M, 256 - (rectangle_size/2)*DISTANCE_1M),
							(256 + (rectangle_size/2)*DISTANCE_1M, 256 + (rectangle_size/2)*DISTANCE_1M)]

			# for coord in self.corners:
				# self.plotted_points.append(self.x_y_to_lat_lon(coord[0], coord[1]))

			# Add the path
			self.path = Path()
			self.path.add_corners(self.corners[0], self.corners[1], self.corners[2], self.corners[3])
			self.path.plan_path()
			for node in self.path.vertices:
				self.plotted_points.append(self.x_y_to_lat_lon(node.x, node.y))
			self.done = True

	def make_map_request(self, lat, lon):
		lat = "%s" % lat
		lon = "%s" % lon
		params = (self.BASE_URL, lat, lon, self.zoom, self.map_height, self.map_height)
		full_url = "%s?center=%s,%s&zoom=%s&size=%sx%s&sensor=false&maptype=satellite" % params
		response = urllib2.urlopen(full_url)
		png_bytes = np.asarray([ord(char) for char in response.read()], dtype=np.uint8)
		cv_array = cv2.imdecode(png_bytes, -1)
		return cv_array

	@property
	def degrees_in_map(self):
		'''
		This logic is based on the idea that zoom=0 returns 360 degrees
		'''
		return (self.map_height / 256.0) * (360.0 / pow(2, self.zoom))

	def degrees_to_meters(self, degrees):
		equator_length_km = 40008
		km_per_degree = equator_length_km / 360.0
		m_per_degree = km_per_degree * 1000
		return degrees * m_per_degree

	@property
	def linear_meters_in_map(self):
		meters_in_map = self.degrees_to_meters(self.degrees_in_map)
		return meters_in_map

	def _window_x_y_to_grid(self, x, y):
		'''
		converts graphical x, y coordinates to grid coordinates
		where (0, 0) is the very center of the window
		'''
		center_x = center_y = self.map_height / 2
		new_x = x - center_x
		new_y = -1 * (y - center_y)
		return new_x, new_y

	def _grid_x_y_to_window(self, x, y):
		center_x = center_y = self.map_height / 2
		new_x = center_x + x
		new_y = center_y - y
		return new_x, new_y

	def x_y_to_lat_lon(self, x, y):
		grid_x, grid_y = self._window_x_y_to_grid(x, y)
		offset_x_degrees = (float(grid_x) / self.map_height) * self.degrees_in_map
		offset_y_degrees = (float(grid_y) / self.map_height) * self.degrees_in_map
		# lat = y, lon = x
		return self.center_lat + offset_y_degrees, self.center_lon + offset_x_degrees

	def lat_lon_to_x_y(self, lat, lon):
		'''
		Returns x, y coordinates where (0, 0) is the top left
		'''
		offset_lat_degrees = lat - self.center_lat
		offset_lon_degrees = lon - self.center_lon
		grid_x = (offset_lon_degrees / self.degrees_in_map) * self.map_height
		grid_y = (offset_lat_degrees / self.degrees_in_map) * self.map_height
		window_x, window_y = self._grid_x_y_to_window(grid_x, grid_y)
		return int(window_x), int(window_y)

	def mouse_callback(self, event, x, y, flag=0, param=None):
		if event == cv2.EVENT_LBUTTONDOWN:
			lat, lon = self.x_y_to_lat_lon(x, y)
			self.plot_point(lat, lon)
		elif event == cv2.EVENT_RBUTTONDOWN:
			self.done = True

	def plot_point(self, lat, lon):
		 self.plotted_points.append((lat, lon,))

	def get_plotted_points_as_x_y_list(self):
		'''
		returns plotted lat, lon points as drawable (x, y) window coordinates
		'''
		return [self.lat_lon_to_x_y(*tuple_inst) for tuple_inst in self.plotted_points]

	def save_path(self):
		
		param1 = "0";
		param2 = "0";
		param3 = "0";
		param4 = "0";
		autocontinue = "1"
		altitude = "40"
		current_waypoint = "0"
		coord = "0"
		command = "16"

		f = open("path.txt", "w+")
		f.write("QGC WPL 110\n")
		count = 0
		for point in self.plotted_points:
			if count == 0:
				current_waypoint = "1"
				altitude = "0"
			else:
				current_waypoint = "0"
				altitude = "40"
			waypoint = "\t".join([str(count), current_waypoint, coord, command, param1, param2, param3, param4, str(point[0]), str(point[1]), altitude, autocontinue])
			f.write(waypoint + "\n")
			count = count + 1
		f.close()

def main():
	# Get data from the map
	geolocator = Nominatim()

	mode = int(raw_input("Enter Mode (0 - Manual, 1 - Autonomous): "))
	if mode != MANUAL_MODE and mode != AUTONOMOUS_MODE:
		print "Error: Wrong Mode Entered"
		return 0

	is_lat_lon = raw_input("Are you entering (lat,lon) coordinates? [n/Y] ").upper()
	if is_lat_lon == "N" or is_lat_lon == "NO":
		address = raw_input("Enter the address: ")
		print "\n"
		location = geolocator.geocode(address)

		if not location:
			print "Incorrect Address"
			return 0
		latitude = location.latitude
		longitude = location.longitude
	else:
		latitude = float(raw_input("Enter latitude: "))
		longitude = float(raw_input("Enter longitude: "))


	map = MapManager(HEIGHT, ZOOM, latitude, longitude, bool(mode))
	# BGR Red
	UNIVERSAL_COLOR = np.array([0,0,255])
	CENTER_COLOR = np.array([255, 0, 0])
	cv2.namedWindow(WINDOW_NAME)
	if mode == MANUAL_MODE:
		cv2.setMouseCallback(WINDOW_NAME, map.mouse_callback)

		while True:
			img = map.static_map
			center = map.lat_lon_to_x_y(map.center_lat, map.center_lon)
			cv2.circle(img, center=center, radius=5, color=CENTER_COLOR, thickness=-1)
			past_point = None
			line_color = np.array([0, 255, 0])
			for (x, y) in map.get_plotted_points_as_x_y_list():
				cv2.circle(img, center=(x, y), radius=5, color=UNIVERSAL_COLOR, thickness=-1)
				if not past_point:
					past_point = (x, y)
				else:
					cv2.line(img, past_point, (x,y), line_color)
					past_point = (x, y)
			cv2.imshow(WINDOW_NAME, img)
			cv2.waitKey(1)
			if map.done:
				break

	else:
		img = map.static_map
		center = map.lat_lon_to_x_y(map.center_lat, map.center_lon)
		cv2.circle(img, center=center, radius=5, color=CENTER_COLOR, thickness=-1)
		past_point = None
		line_color = np.array([0, 255, 0])
		for (x, y) in map.get_plotted_points_as_x_y_list():
			cv2.circle(img, center=(x, y), radius=5, color=UNIVERSAL_COLOR, thickness=-1)
			if not past_point:
				past_point = (x, y)
			else:
				cv2.line(img, past_point, (x,y), line_color)
				past_point = (x, y)
		cv2.imshow(WINDOW_NAME, img)
		cv2.waitKey(1)
		
	while(True):
		confirmed = raw_input("Is the path correct [y/N]: ").upper()
		if confirmed == "N" or confirmed == "NO":
			print "Path discarded"
			break
		elif confirmed == "Y" or confirmed == "YES":
			print "Saving Path..."
			print "\n"
			map.save_path()
			print map.plotted_points
			break


if __name__ == "__main__":
	main()
