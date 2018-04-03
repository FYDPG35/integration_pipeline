from multiprocessing import Queue, Process
# import requests
import subprocess
import datetime
import zipfile
import sys, os, shutil
import argparse

home = "/home/ubuntu"
ODM_PATH = "/home/ubuntu/OpenDroneMap"
FILTERING_PATH = "/home/ubuntu/filtering/build"

class Mapper:
		
	def init(self, image_zip, filtering):
		self.image_zip = image_zip
		self.filtering = filtering

		# First, establish the new project directory
		date_time = datetime.datetime.now()
		self.project_name = "project_" + "_".join([str(date_time.year), str(date_time.month), str(date_time.day), str(date_time.hour), str(date_time.minute), str(date_time.second)])
		# self.project_name = "project_2018_3_20_1_5_5"
		self.project_dir = home + "/projects/" + self.project_name

		self.images_dir = self.project_dir + "/images/"
		# return True
		# Unzip the content
		if not zipfile.is_zipfile(self.image_zip):
			print "Error: Bad Zipfile!"
			return False

		zip_ref = zipfile.ZipFile(self.image_zip, 'r')
		zip_ref.extractall(self.images_dir)
		zip_ref.close()

		image_folder = self.image_zip.split('.')[0].split("/")[-1] # Get the name of the image folder
		zip_folder = self.images_dir + image_folder + "/"
		print zip_folder
		print self.images_dir
		files = os.listdir(zip_folder)

		for f in files:
        		shutil.move(zip_folder+f, self.images_dir)
		os.rmdir(self.images_dir + image_folder) # Remove the initial folder

		print "\n\nProject is established at: %s" % self.project_dir
		print "Images are at: %s\n\n" % self.images_dir

		return True
	
	def run_ODM(self):
		""" Run ODM in this function """
		# os.chdir(ODM_PATH)
		run_command = ODM_PATH + "/run.sh"
		subprocess.call([run_command, self.project_name])


	def run_Filtering(self):
		if self.filtering:
			""" Run filtering here """
			print "*************************"
			print "    Filtering Enabled"
			print "*************************"
			run_command = FILTERING_PATH + "/filter"
                	subprocess.call([run_command, self.project_dir + "/opensfm/depthmaps/merged.ply", self.project_dir])
			self.obj_path = self.project_dir + "/odm_texturing/odm_texturing/odm_textured_model.stl"
		else:
			print "**************************"
                        print "    Filtering Disabled"
                        print "**************************"
			self.obj_path = self.project_dir + "/odm_texturing/odm_texturing/odm_textured_model.obj"

	def post_data(self):
		orthophoto_path = self.project_dir + "/odm_orthophoto/odm_orthophoto.png"
		files = {'upload_file': open(orthophoto_path,'rb')}
		url = "http://"
		# values = {'DB': 'photcat', 'OUT': 'csv', 'SHORT': 'short'}
		# r = requests.post(url, files=files)

def main():
	parser = argparse.ArgumentParser(description="Integration Executable for running ODM, Post-Processing Filtering and Website updates")
	parser.add_argument("zipfile", help="Path to zipfile")
	parser.add_argument("--filter", help="Enable Filtering", action="store_true")
	parsed_args = parser.parse_args()

	mapper = Mapper()
	if not mapper.init(parsed_args.zipfile, parsed_args.filter):
		return
	print "*********************"
    print "    Initialized"
    print "*********************"
	p = Process(target=mapper.run_ODM())
	p.start()
	p.join() # this blocks until the process terminates

	p = Process(target=mapper.run_Filtering())
	p.start()
	p.join() # this blocks until the process terminates


if __name__ == "__main__":
	main()
