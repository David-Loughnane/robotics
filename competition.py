# MAC Address 74:da:38:5d:ff:0c
import time
import random
import math
import brickpi
import sys
import os
import numpy as np


SLEEP_TIME = 4.50
WAYPOINT_THRESHOLD = 2
REF_ANGLE = 14.65        # reference angle is 40cm
REF_CM = 40.0
MAP_OFFSET = 220.00      # height of map + 10 buffer

# PARTICLES
NUM_PARTICLES = 100
PART_NOISE_E = 0.02      # XY variance 1cm straight (~40cm std dev 1cm)
PART_NOISE_F = 0.000006  # Angular variance 1cm straight (~1 degree std dev for 1cm)
PART_NOISE_G = 0.002     # Angular variance 1 radian turn (~4.5 degree std dev for 90 degree turn)

# LIKELIHOOD 
FAT_TAIL_FACTOR = 0.05  
BETA_MIN = -0.785
BETA_MAX = +0.785

# ROBOT
SONAR_BIAS = 3      # sonar underestimates distance by 3cm
SONAR_STD_DEV = 2   # ANY THOUGHTS ON THIS NUMBER??
SON_PORT = 3
SON_READINGS = 10
SON_SLEEP = 0.05
SONAR_MAX = 255.00
SONAR_MIN = 20.00
MOTORS = [2,3]
ROT_CONVERSION = 4.75/1.5708

# IF YOU CHANGE NUM_DEGREES_TO_TURN CONSTANT YOU NEED TO REWRITE ALL REFERENCE DATA
NUM_DEGREES_TO_TURN = 1
ONE_DEGREE = (2*math.pi) / 360 #one degree in radians
LOWER_SONAR_BOUND = 20  # check this
UPPER_SONAR_BOUND = 130 #check this
SHIFT_STEP = NUM_DEGREES_TO_TURN * ONE_DEGREE #

NUM_BINS = 41
BIN_SIZE = float(UPPER_SONAR_BOUND - LOWER_SONAR_BOUND) / float(NUM_BINS)



# PARTICLE CLASS ############################################################
class Particles:

    def __init__(self):
        p_weight = 1/float(NUM_PARTICLES)
        self.data = []
        for i in range(NUM_PARTICLES):
            self.data.append([0.0, 0.0, 0.0, p_weight])
    
# ROBOT CLASS ###############################################################
class Robot:
    
    def __init__(self):       
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.particles = Particles()

        # Set up interfaces for sonar and motors
        self.interface = brickpi.Interface()
        self.interface.initialize()
        self.interface.sensorEnable(SON_PORT,brickpi.SensorType.SENSOR_ULTRASONIC);
        self.interface.motorEnable(MOTORS[0])
        self.interface.motorEnable(MOTORS[1])        
        # motor parameters
        self.motorParams = self.interface.MotorAngleControllerParameters()
        self.motorParams.maxRotationAcceleration = 6.0
        self.motorParams.maxRotationSpeed = 12.0
        self.motorParams.feedForwardGain = 255/20.0
        self.motorParams.minPWM = 18.0
        self.motorParams.pidParameters.minOutput = -255
        self.motorParams.pidParameters.maxOutput = 255
        #PID
        self.motorParams.pidParameters.k_p = 270.0
        self.motorParams.pidParameters.k_i = 300.0
        self.motorParams.pidParameters.k_d = 13.5
        self.interface.setMotorAngleControllerParameters(MOTORS[0],self.motorParams)
        self.interface.setMotorAngleControllerParameters(MOTORS[1],self.motorParams)
        

    def set(self, new_x, new_y, new_theta):
        if new_x < 0:
            raise ValueError, 'New robot x coordinate value is invalid'
        if new_y < 0:
            raise ValueError, 'New robot x coordinate value is invalid'
        self.x = float(new_x)
        self.y = float(new_y)
        self.theta = new_theta
        if(new_theta < 0):
            self.theta = abs(self.theta) % (2 * math.pi)
            self.theta = (-1) * self.theta
        else:
            self.theta = (self.theta)%(2*math.pi)


    def setAllParticles(self, new_x, new_y, new_theta):
        if new_x < 0.0:
            raise ValueError, 'New robot x coordinate value is invalid'
        if new_y < 0.0:
            raise ValueError, 'New robot x coordinate value is invalid'
        for p in self.particles.data:
            p[0] = float(new_x)
            p[1] = float(new_y)
            p[2] = new_theta % (2 * math.pi)

 
    def printParticles(self):            
        #print self.particles.data
        # Y coordinates inverted and offset by 220
        particleTuples = [(particle[0], 
                (particle[1]*-1) + MAP_OFFSET, 
                particle[2]) 
            for particle in self.particles.data]
        print 'drawParticles:', str(particleTuples)


    def mcl_movement(self, X, Y):          
        # do-while loop, until within waypoint distance threshold 
        while True:
            self.navigateToWayPoint(X,Y)
      
            measurement = self.sleepmeasureSonar()
            self.update_and_normalise(measurement)
            self.resample()
            self.normalise_only()

            estimates = self.particleEstimate()
            self.set(estimates[0],estimates[1],estimates[2])

            waypoint_distance = math.sqrt(pow(X - self.x, 2) + pow(Y - self.y, 2))
            print 'Remaining distance to waypoint: ', '%.2f' % waypoint_distance
            if (waypoint_distance <= WAYPOINT_THRESHOLD):
                print '-----------------------------------------'
                print 'Arrived at Waypoint!'
                break


    def navigateToWayPoint(self, X, Y):
        # Calculate distance and angle to move next
        xPrime = X-self.x
        yPrime = Y-self.y
        print 'Move x: ', '%.2f' % xPrime, ', Move y: ', '%.2f' % yPrime

        phi = math.atan2(yPrime, xPrime) - self.theta
        if (abs(phi) > math.pi):
            if (phi > 0):
                phi = -(math.pi*2 - phi)    #positive (-90deg instead of 270deg)
            else:
                phi = math.pi*2 + phi       #Negative (90deg instead of -270deg)

        D = math.sqrt(pow(xPrime, 2) + pow(yPrime, 2))
        print 'Turn angle: ', '%.2f' % phi, ', go straight: ', '%.2f' % D

        self.turn(phi)
        self.updateParticlesTurn(phi)   
        time.sleep(SLEEP_TIME)     

        self.moveStraight(D)
        self.updateParticlesStraight(D)
        time.sleep(SLEEP_TIME)


    def turn(self, angle):    
        wheel_rotation_angle = angle * (ROT_CONVERSION)
        self.interface.increaseMotorAngleReferences(MOTORS, [(-1.0)*wheel_rotation_angle, wheel_rotation_angle])            
 

    def updateParticlesTurn(self, phi):
        for particle in self.particles.data:
            g = random.gauss(0.0, math.sqrt(abs(phi) * PART_NOISE_G))
            particle[2] += phi + g
        self.printParticles() 


    def moveStraight(self, distance):    
        wheel_rotation_angle = (REF_ANGLE / REF_CM) * distance
        self.interface.increaseMotorAngleReferences(MOTORS, [wheel_rotation_angle, wheel_rotation_angle])
        

    def updateParticlesStraight(self, D):      
        for particle_up in self.particles.data:
            e = random.gauss(0, (math.sqrt(D * PART_NOISE_E)))
            f = random.gauss(0, (math.sqrt(D * PART_NOISE_F)))
            particle_up[0] += ((D+e)*math.cos(particle_up[2]))
            particle_up[1] += ((D+e)*math.sin(particle_up[2]))
            particle_up[2] += f
        self.printParticles()    


    def measureSonar(self):
        measurements = []
        for i in range(SON_READINGS):
            usReading = self.interface.getSensorValue(SON_PORT)
            if usReading:
                measurements.append(usReading[0])            
            #time.sleep(SON_SLEEP)
    
        if len(measurements) > 0 :
            measurement = np.median(measurements)
            #print 'Sonar measurement: ', measurement 
            if ((measurement < SONAR_MIN) or (measurement >= SONAR_MAX)):
                #print "Warning: bad sonar readings"
                return -1
            measurement -= SONAR_BIAS    # adjust for bias in measurement
            #print 'Sonar Value: ', '%.2f' % measurement    
            return measurement
        else:
            #print "Warning: bad sonar readings"
            return -1
        
    def sleepmeasureSonar(self):
        measurements = []
        for i in range(SON_READINGS):
            usReading = self.interface.getSensorValue(SON_PORT)
            if usReading:
                measurements.append(usReading[0])            
            time.sleep(SON_SLEEP)
    
        if len(measurements) > 0 :
            measurement = np.median(measurements)
            print 'Sonar measurement: ', measurement 
            if ((measurement < SONAR_MIN) or (measurement >= SONAR_MAX)):
                #print "Warning: bad sonar readings"
                return -1
            measurement -= SONAR_BIAS    # adjust for bias in measurement
            print 'Sonar Value: ', '%.2f' % measurement    
            return measurement
        else:
            #print "Warning: bad sonar readings"
            return -1


    def update_and_normalise(self, measurement):
        # Update weighting based on likelihood
        sumWeight = 0.0
        for p in self.particles.data:
            p_likelihood = self.calculate_likelihood(p[0],p[1],p[2], measurement)
            # don't modify particle likelihood if likelihood is -1
            if(p_likelihood != -1):
                p[3] *= p_likelihood
                sumWeight += p[3]
            else:
                #print 'Error: particle likelihood returned as -1. Particle left unchanged'
                sumWeight += p[3]

        newSum = 0.0
        for p in self.particles.data:
            p[3] /= sumWeight
            newSum += p[3]
        if ( (abs(newSum - 1.0) > 0.001) ):
            print 'ERROR, SUM AFTER NORMALISATION != 1, SUM = ', newSum
         

    def normalise_only(self):
        sumWeight = 0.0
        for p in self.particles.data:
                sumWeight += p[3]

        newSum = 0.0
        for p in self.particles.data:
            p[3] /= sumWeight
            newSum += p[3]
        if ( (abs(newSum - 1.0) > 0.001) ):
            print 'ERROR, SUM AFTER NORMALISATION != 1, SUM = ', newSum
       

    def resample(self):
        new_particles = []    
        cum_weight = []
        length = 0

        # calculate cumulative weight array
        for p in self.particles.data:
            if length == 0:
                new_weight = p[3]
            else:
                new_weight = p[3] + cum_weight[length-1]  
            cum_weight.append(new_weight)
            length += 1  

        for i in range(0,len(cum_weight)):
            random_num = random.uniform(0.0,1.0)
            # find where weight lies in array, add copy of particle
            for j in range(0,len(cum_weight)):
                if cum_weight[j] >= random_num:
                    new_particles.append(list(self.particles.data[j]))
                    break
        self.particles.data = new_particles[:]   


    def particleEstimate(self): 
        estimate = [0.0, 0.0, 0.0]
        for particle in self.particles.data:
            estimate[0] += particle[0]*particle[3]
            estimate[1] += particle[1]*particle[3]
            estimate[2] += particle[2]*particle[3]
        print 'Updated Estimate', estimate
        return estimate

    
    def calculate_likelihood(self, x, y, theta, z): 
        wall, m = self.find_walls(x, y, theta, z)
        if (wall == [] or m == -1):
            return -1
        likelihood = self.find_likelihood(m, z, theta, wall)
        return likelihood

    
    # calc likelihood on depth and readings
    # as well as the max incidence angle
    def find_likelihood(self, m, z, theta, wall):
        likelihood = 0.0
        diff = float(z - m)
        power = (-1 * (math.pow(diff, 2))) / (2 * (math.pow(SONAR_STD_DEV, 2)))  ### MODIFY !!! PART_NOISE NOW SCALES!!! ###

        # calculates e^power
        likelihood = math.exp(power) + FAT_TAIL_FACTOR

        ax = wall[0]
        ay = wall[1]
        bx = wall[2]
        by = wall[3]

        beta_num = math.cos(theta)*(ay-by) + math.sin(theta)*(bx -ax)
        beta_den = math.sqrt( (ay-by)**2 + (bx - ax)**2 )
        beta = math.acos(beta_num/beta_den)
        #beta is returning as 1.57 radian when perpendicular to wall
        # needs to be between 0.785 and 2.356 radians for accurate reading 
        # can't get rid of this function altogether
        if (beta < BETA_MIN or beta > BETA_MAX):
            likelihood = -1        
        return likelihood

    
    # find the (one) wall for particle collision
    def find_walls(self, x1, y1, theta1, z1):
        valid_walls = []
        finalM = float('inf')
        for wall in wall_array:
            ax = wall[0]
            ay = wall[1]
            bx = wall[2]
            by = wall[3]
            # calculate the ground truth value for the particle with given wall
            m_num = (by - ay) * (ax - x1) - (bx - ax)* (ay -y1) 
            m_denom = (by - ay) * math.cos(theta1) - (bx - ax)*math.sin(theta1)

            if m_denom == 0.0: 
                continue
            else:
                m = (m_num / m_denom)
                # find the collision for the particle with the wall                
                if (m > 0.0):
                    collision = [x1 + m*math.cos(theta1), y1 + m*math.sin(theta1)]
                    # Append wall if the particle's collision point intersects with wall
                    if self.is_valid(collision, ax,ay,bx,by):
                        if m > 0.0:
                            valid_walls.append([wall, m, abs(m-z1)])
        if(valid_walls==[]):
            return [], -1
        else:
            best_wall = valid_walls[0][0]
            best_m = valid_walls[0][1]
            best_diff = valid_walls[0][2]

            for wall in valid_walls:
                if (wall[2] < best_diff):
                    best_wall = wall[0]
                    best_m = wall[1]
                    best_diff = wall[2]
            return best_wall, best_m


    def is_valid(self, collision, ax, ay, bx, by):
        if ((collision[0] <= bx and collision[0] >= ax) or (collision[0] <= ax and collision[0] >= bx)):
            if ((collision[1] <= by and collision[1] >= ay) or (collision[1] <= ay and collision[1] >= by)):
                return True      
        return False
    
    
#############################################################################################################

# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 360):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# Location signature class: stores a signature characterizing one location
class LocationHistogram:
    def __init__(self, no_bins = NUM_BINS):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        self.histograms = [];
        self.obsSig = 'obssig.dat'
        self.obsSigHist = 'obshist.dat'
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('oneDegreeloc_{0:02d}.dat'.format(i))
            self.histograms.append('oneDegreehist2_{0:02d}.dat'.format(i))
            
    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();
        
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def saveHistogram(self, signature, index):
        filename = self.histograms[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();
    
    
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def saveObs(self, signature):
        filename = self.obsSig
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();
        
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def saveObsHistogram(self, signature):
        filename = self.obsSigHist
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();


    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return ls

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def readHistogram(self, index):
        histogram = LocationHistogram()
        filename = self.histograms[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(histogram.sig)):
                s = f.readline()
                if (s != ''):
                    histogram.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return histogram
    
# FILL IN: spin robot or sonar to capture a signature and store it in ls

# Initialize location signature
def characterize_location(ls):
    initialPosition = 0.0

    # find initial wheel rotation
    # find delta between current and initial
    motorAngles = bug.interface.getMotorAngles(MOTORS)
    initialPosition = motorAngles[1][0]
    
    currentPosition = initialPosition
    delta = initialPosition - currentPosition  
    
    print "initial ", initialPosition, " current ", currentPosition
    
    i = 0
    bug.turn(2*math.pi)
    while not bug.interface.motorAngleReferencesReached(MOTORS):
        motorAngles = bug.interface.getMotorAngles(MOTORS)
        currentPosition = motorAngles[1][0]
        delta = currentPosition - initialPosition
        
        if (delta > .05277 * i) and i < 360:
            reading = int(bug.measureSonar())

            if ((reading > LOWER_SONAR_BOUND) and (reading < UPPER_SONAR_BOUND)):
                ls.sig[i] = reading            
            else:
                ls.sig[i] = 0

            i += 1


def convert_to_frequency(ls, histogram):
    # calculte the data see pseudocode
    for index in range(len(ls.sig)):
        number = ls.sig[index]
        if ls.sig[index] > 0:
            for binIndex in range(0,len(histogram.sig)):
                upperBound = (float(binIndex + 1)  * BIN_SIZE) + LOWER_SONAR_BOUND 
                if number < upperBound:                
                    histogram.sig[binIndex] +=1
                    break

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    histogram = LocationHistogram()
    
    characterize_location(ls)
    convert_to_frequency(ls, histogram)
    histogram.print_signature()
    
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    signatures.saveHistogram(histogram,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

    
def compare_signatures(ls1, ls2):
    totalDiff = 0
    
    for index in range(len(ls1.sig)): 
        difference = ls1.sig[index] - ls2.sig[index]  
        difference = math.pow(difference,2)
        totalDiff += difference
    
    print "calculated in compare signatures", totalDiff
    return totalDiff
            
def compare_signatures_shift(ls1, ls2, shift):
    totalDiff = 0
    n = 360/NUM_DEGREES_TO_TURN
    for index in range(n):
        relativeShift = (index+shift) % n
        difference = ls1.sig[relativeShift] - ls2.sig[index]
        difference = math.pow(difference,2)
        totalDiff += difference
    
    return totalDiff


def recognize_orientation(ls1,ls2):
    orientations = []
    currentMin = 10000000
    currentIndex = 0
    
    numShifts = 360/NUM_DEGREES_TO_TURN
    
    for shift in range(numShifts):
    # for every possible sift
        diff = compare_signatures_shift(ls1, ls2, shift)
        orientations.append(diff)
        #print "index ", shift, " difference ", diff
        
    for index in range(len(orientations)):
        
        if orientations[index] < currentMin:
            currentMin = orientations[index]
            currentIndex = index 
    #need to check for > 2pi
    print "min index is ", currentIndex
    shift = (currentIndex * SHIFT_STEP)       #CHECK 90 DEGREES HERE
    print "shift is ", shift # this gives how far away from original reading need to take 2pi - theta to get actual offset from 0
    if(shift > math.pi):
        theta = 2*math.pi - shift #TESTTESTTEST
    else:
        theta = (-1.0)*shift
    
    print "theta is ", theta
    #bug.turn((-1.0)*theta)
    #time.sleep(3)
    return theta

    
# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen

def recognize_location():
    ls_obs = LocationSignature();
    histogram_obs = LocationHistogram();
    
    characterize_location(ls_obs);
    convert_to_frequency(ls_obs, histogram_obs);
    
    signatures.saveObs(ls_obs)
    signatures.saveObsHistogram(histogram_obs)
    
    identifiedIndex = 0;
    identifiedDiff = 100000;
    
    distOne = 0 
    distTwo = 0
    
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        histogram_read = signatures.readHistogram(idx);
        difference = compare_signatures(histogram_obs, histogram_read)
        
        if difference < identifiedDiff:
            identifiedIndex = idx
            identifiedDiff = difference


        if idx == 1:
            distOne = difference
        
        if idx == 2:
            distTwo = difference
    
    
    
    if identifiedIndex == 1:
        ratioOneTwo = distTwo/distOne
        if ratioOneTwo < 10.0:
            identifiedIndex == 2
            print "wrote over one!"
            
    print "We are at ", identifiedIndex
    
    lsReference = LocationSignature()
    lsReference = signatures.read(identifiedIndex)
    
    return ls_obs, lsReference, identifiedIndex

    
lineOA = (0,220,0,52)
lineAB = (0,52,84,52)
lineBC = (84,52,84,10)
lineBD = (84,52,84,10)
lineDE = (84,10,168,10)
lineEF = (168,10,168,136)
lineFG = (168,136,210,136)
lineGH = (210,136,210,220)
lineHO = (210,220,0,220)

print "drawLine:" + str(lineOA)
print "drawLine:" + str(lineAB)
print "drawLine:" + str(lineBC)
print "drawLine:" + str(lineBD)
print "drawLine:" + str(lineDE)
print "drawLine:" + str(lineEF)
print "drawLine:" + str(lineFG)
print "drawLine:" + str(lineGH)
print "drawLine:" + str(lineHO)

lineOA_a = [0,0,0,168]
lineAB_a = [0,168,84,168]
lineBC_a = [84,168,84,126]
lineBD_a = [84,168,84,210]
lineDE_a = [84,210,168,210]
lineEF_a = [168,210,168,84]
lineFG_a = [168,84,210,84]
lineGH_a = [210,84,210,0]
lineHO_a = [210,0,0,0]

wall_array = []
wall_array.append(lineOA_a)
wall_array.append(lineAB_a)
wall_array.append(lineBC_a)
wall_array.append(lineBD_a)
wall_array.append(lineDE_a)
wall_array.append(lineEF_a)
wall_array.append(lineFG_a)
wall_array.append(lineGH_a)
wall_array.append(lineHO_a)







# MAIN PROGRAM ##############################################################

bug = Robot()

signatures = SignatureContainer(5)
# signatures.delete_loc_files()
# bug.turn(2*math.pi)
# currentVersion.time.sleep(5)
# learn_location();
obsLocation, referenceLocation, index = recognize_location()
theta_start = recognize_orientation(obsLocation, referenceLocation)
#x_start, y_start = mapIndex(index)

print "theta_start is ", theta_start

waypoint = index
initial_theta = theta_start 

if waypoint == 0:
    bug.set(84, 30, initial_theta)
    bug.setAllParticles(84, 30, initial_theta)
elif waypoint == 1:
    bug.set(180, 30, initial_theta)
    bug.setAllParticles(180, 30, initial_theta)
elif waypoint == 2:
    bug.set(180, 54, initial_theta)
    bug.setAllParticles(180, 54, initial_theta)
elif waypoint == 3:
    bug.set(138, 54, initial_theta)
    bug.setAllParticles(138, 54, initial_theta)
elif waypoint == 4:
    bug.set(138, 168, initial_theta)
    bug.setAllParticles(138, 168, initial_theta)
bug.printParticles()

NUM_MOVEMENTS = 5

## ANTI-CLOCKWISE
while (NUM_MOVEMENTS > 0):
    
    if waypoint == 0:
        bug.mcl_movement(180, 30)
    elif waypoint == 1:
        bug.mcl_movement(180, 54)
    elif waypoint == 2:
        bug.mcl_movement(138, 54)
    elif waypoint == 3:
        bug.mcl_movement(138, 168)
    elif waypoint == 4:
        bug.mcl_movement(84, 30)
    
    print waypoint
    '''
    ## CLOCKWISE
    while (NUM_MOVEMENTS > 0):
    if waypoint == 0:
        bug.mcl_movement(84,84)
    elif waypoint == 8:
        bug.mcl_movement(138,84)
    elif waypoint == 7:
        bug.mcl_movement(138,168)
    elif waypoint == 6:
        bug.mcl_movement(138,84)     
    elif waypoint == 5:
        bug.mcl_movement(138,54) 
    elif waypoint == 4:
        bug.mcl_movement(180,54) 
    elif waypoint == 3:
        bug.mcl_movement(180,30)
    elif waypoint == 2:
        bug.mcl_movement(138,30)
    else:
        bug.mcl_movement(84,30)
    '''
    waypoint = (waypoint + 1) % 5
    NUM_MOVEMENTS -= 1
    print 'Waypoints left: ', NUM_MOVEMENTS, 'Next waypoint: ', waypoint
    print '-----------------------------------------'
    if NUM_MOVEMENTS == 0:
        print 'FINAL DESTINATION REACHED!'




