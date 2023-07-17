import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        self.x=np.zeros(np.size(lower_bound))
        delta = upper_bound - lower_bound
        self.v=np.zeros(np.size(delta))
        self.best=np.zeros(np.size(lower_bound))
        self.best_value=-inf



class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):

        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.delta=upper_bound-lower_bound


        self.w=hyperparams.inertia_weight
        self.phip=hyperparams.cognitive_parameter
        self.phig=hyperparams.social_parameter
        self.num_particles=hyperparams.num_particles


        self.i=0
        self.particles=[]

        i=0

        while i<self.num_particles:
            self.particles.append(Particle(lower_bound,upper_bound))
            self.particles[i].x=np.random.uniform(lower_bound,upper_bound)
            self.particles[i].v=np.random.uniform(-self.delta,self.delta)

            i += 1


        self.best_position=np.zeros(np.size(upper_bound))
        self.best_value=-inf


    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """

        return self.best_position


    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        return self.best_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        rp=random.uniform(0.0,1.0)
        rg=random.uniform(0.0,1.0)

        self.particles[self.i].v = self.w * self.particles[self.i].v + self.phip * rp * (self.particles[self.i].best - self.particles[self.i].x) + self.phig * rg * (self.best_position - self.particles[self.i].x)

        for i in range(len(self.delta)):
            self.particles[self.i].v[i]=min(max(self.particles[self.i].v[i],-self.delta[i]),self.delta[i])

        self.particles[self.i].x=self.particles[self.i].x+self.particles[self.i].v

        for i in range(len(self.upper_bound)):
            self.particles[self.i].x[i] = min(max(self.lower_bound[i], self.particles[self.i].x[i]), self.upper_bound[i])


        return self.particles[self.i].x

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        self.i+=1
        if self.i>= self.num_particles:
            self.i=0


    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.

        """

        if value > self.best_value:
            self.best_position=self.particles[self.i].x
            self.best_value=value

        if value > self.particles[self.i].best_value:
            self.particles[self.i].best=self.particles[self.i].x
            self.particles[self.i].best_value=value


        self.advance_generation()


