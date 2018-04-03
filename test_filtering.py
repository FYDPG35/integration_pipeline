import cv2
import numpy as np
from Queue import Queue

def inbound(i, max_val):
	return i < max_val and i >= 0

def psnr(img_f, img_g):
	max_f = 255.0 # 8-bit image
	img_f = np.divide(img_f.astype(float),max_f)
	img_g = np.divide(img_g.astype(float),max_f)
	mse = (1.0 / (img_f.shape[0] * img_f.shape[1])) * np.sum(np.power(np.subtract(img_f, img_g),2))
	return 10*np.log10(float(max_f**2/float(mse)))

def noisy(image, var):
	row,col= image.shape
	mean = 0
	sigma = var**0.5
	gauss = np.random.normal(mean,sigma,(row,col))
	gauss = gauss.reshape(row,col)
	noisy = image + gauss
	noisy = noisy.astype(np.uint8)
	return noisy

def pad_canny(canny, num_pixels):
	new_canny = np.zeros((canny.shape[0], canny.shape[1]))
	for i in range(0, canny.shape[0]):
		for j in range(0, canny.shape[1]):
			for k in range(1,num_pixels+1):
				if canny[i][j]:
					if (i-1) >= 0:
						new_canny[i-k][j] = 1
					if (i+1) > canny.shape[0]:
						new_canny[i+k][j] = 1
					if (j-1) >= 0:
						new_canny[i][j-k] = 1
					if (j+1) > canny.shape[0]:
						new_canny[i][j+k] = 1
	return canny

def apply_mean(noisy, canny, i, j, window):
	value = 0
	count = 0
	# left = right = up = down = True
	width =  noisy.shape[0]
	height = noisy.shape[1]

	queue = Queue()
	queue.put((i,j))
	while (not queue.empty() and count < window):
		(k,m) = queue.get()
		if canny[k][m]:
			continue
		if inbound(k+1, width):
			if inbound(m-1, height) and not canny[k+1][m-1]:
				queue.put((k+1,m-1))
			if inbound(m, height) and not canny[k+1][m]:
				queue.put((k+1,m))
			if inbound(m+1, height) and not canny[k+1][m+1]:
				queue.put((k+1,m+1))
		if inbound(k-1, width):
			if inbound(m-1, height) and not canny[k-1][m-1]:
				queue.put((k-1,m-1))
			if inbound(m, height) and not canny[k-1][m]:
				queue.put((k-1,m))
			if inbound(m+1, height) and not canny[k-1][m+1]:
				queue.put((k-1,m+1))
		if inbound(m+1, height) and not canny[k][m+1]:
			queue.put((k, m+1))
		if inbound(m-1, height) and not canny[k][m-1]:
			queue.put((k, m-1))
		value = value + noisy[k][m]
		count  = count + 1
	print count
	# while ((left or right or up or down) and count < window):
	# 	print "i + inc: %d" % (i + inc)
	# 	if right and (i + inc) < noisy.shape[0] and (j + inc - 1) < noisy.shape[1] and not canny[i + inc][j + inc - 1]:
	# 		value = value + noisy[i + inc][j+inc - 1]
	# 		count = count + 1
	# 	else:
	# 		right  = False
	# 	print "i - inc: %d" % (i - inc)
	# 	if left and (i - inc) >= 0 and (j - inc + 1) < noisy.shape[1] and not canny[i - inc][j - inc + 1]:
	# 		value = value + noisy[i + inc][j+inc - 1]
	# 		count = count + 1
	# 	else:
	# 		left  = False
	# 	print "j + inc: %d" % (j + inc)
	# 	if up and (j + inc) < noisy.shape[1] and not canny[i + inc - 1][j+inc]:
	# 		value = value + noisy[i + inc][j+inc - 1]
	# 		count = count + 1
	# 	else:
	# 		up  = False
	# 	print "j - inc: %d" % (j - inc)
	# 	if down and (i + inc) >= 0 and not canny[i - inc + 1][j-inc]:
	# 		value = value + noisy[i + inc][j+inc - 1]
	# 		count = count + 1
	# 	else:
	# 		down  = False
	# 	inc = inc + 1
	# for k in range(0,window/2):
	# 	if (i + k) >= noisy.shape[0] or canny[i+k][j]:
	# 		continue
	# 	# Going forward
	# 	for m in range(0,window/2):
	# 		if (j + m) >= noisy.shape[1] or canny[i+k][j+m]:
	# 			break
	# 		# value = value + (window**2 - m*k)*float(noisy[i+k][j+m])
	# 		value = value + float(noisy[i+k][j+m])
	# 		# count = count + (window**2 - m*k)
	# 		count = count + 1
	# 	# Going backwards
	# 	for m in range(0,window/2):
	# 		if (j - m) >= noisy.shape[1] or canny[i+k][j-m]:
	# 			break
	# 		# value = value + (window**2 - m*k)*float(noisy[i+k][j-m])
	# 		value = value + float(noisy[i+k][j-m])
	# 		# count = count + (window**2 - m*k)
	# 		count = count + 1

	# for k in range(0,window/2):
	# 	if (i - k) >= noisy.shape[0] or canny[i-k][j]:
	# 		continue
	# 	# Going forward
	# 	for m in range(0,window/2):
	# 		if (j + m) >= noisy.shape[1] or canny[i-k][j+m]:
	# 			break
	# 		# value = value + (window**2 - m*k)*float(noisy[i-k][j+m])
	# 		value = value + float(noisy[i-k][j+m])
	# 		# count = count + (window**2 - m*k)
	# 		count = count + 1
	# 	# Going backwards
	# 	for m in range(0,window/2):
	# 		if (j - m) >= noisy.shape[1] or canny[i-k][j-m]:
	# 			break
	# 		# value = value + (window**2 - m*k)*float(noisy[i-k][j-m])
	# 		value = value + float(noisy[i-k][j-m])
	# 		# count = count + (window**2 - m*k)
	# 		count = count + 1
	# if count is not 16:
	# 	print count
	return value/float(count)

def adaptiveBilateralFilter(original_img, noisy_img, window_size):
	# canny_img = cv2.Canny(noisy_img, 100, 200)
	canny_img = cv2.Canny(original_img, 100, 200)
	canny = pad_canny(canny_img, 3)
	cv2.imwrite('/Users/hugolouisseize/Documents/MATLAB/Filtering/images/canny_image.png', canny)
	# cv2.imshow("canny", canny)
	# cv2.waitKey(0)
	filtered_img = np.zeros((noisy_img.shape[0], noisy_img.shape[1]))
	for i in range(0, noisy_img.shape[0]):
		for j in range(0, noisy_img.shape[0]):
			if canny_img[i][j]:
				# This is an edge so keep
				filtered_img[i][j] = noisy_img[i][j]
			else:
				# Average the values of a maximum window size that is within edges
				filtered_img[i][j] = apply_mean(noisy_img, canny_img, i, j, window_size)
	return filtered_img.astype(np.uint8)

	
def test_ABF(original_img):
	print "*********************************"
	print "Test of Adaptive Bilateral Filter"
	print "*********************************"

	var = 2 # Gaussian Variance
	noisy_img = noisy(original_img, var)
	noisy_psnr = psnr(original_img, noisy_img)
	# Apply Adaptive Bilateral Filter
	filtered_img = adaptiveBilateralFilter(original_img, noisy_img, 25**2)
	filtered_psnr = psnr(original_img, filtered_img)
	# filtered_img = noisy(original_img, 2)
	print "\nNoisy PSNR: %f" % noisy_psnr
	print "\nFiltered PSNR: %f" % filtered_psnr
	cv2.imshow('Noisy',noisy_img)
	cv2.imshow('Filtered',filtered_img)

	cv2.waitKey(0)
	return noisy_img, filtered_img

def main():
	img = cv2.imread('test_image.png', cv2.IMREAD_GRAYSCALE)
	print img.shape
	print img[0][0]
	noisy, filtered = test_ABF(img)
	cv2.imwrite('/Users/hugolouisseize/Documents/MATLAB/Filtering/images/noisy_image.png', noisy)
	cv2.imwrite('/Users/hugolouisseize/Documents/MATLAB/Filtering/images/filtered_image.png', filtered)
	
if __name__ == "__main__":
	main()