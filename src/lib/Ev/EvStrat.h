#ifndef EVSTRAT_H
#define EVSTRAT_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Ev/rng.h"
#include "Ev/rng.c"


typedef struct genome_s genome_t;
typedef struct population_s population_t;

void ES_Genome_Malloc(genome_t *gemone, int length);
void ES_Genome_Free(genome_t *genome);
void ES_Population_Malloc(population_t *population, int member_count, int genome_length);

void ES_Genome_Print(genome_t *genome);
void ES_Population_Print(population_t *population);

void ES_Genome_Copy(genome_t *source_genome, genome_t *festination_genome);
void ES_Population_Copy(population_t *source_population, population_t *destination_population);

void ES_Genome_Init(genome_t *genome, double min, double max, RNG *rng);
void ES_Population_Init(population_t *population, double min, double max, RNG *rng);

void ES_Genome_X_Mutate(genome_t *genome, RNG *rng);
void ES_Genome_Sigma_Mutate(genome_t *genome, double tau_prime, double tau, double sigma_epsilon, RNG *rng);
void ES_Population_Mutate(population_t *population, RNG *rng);

void ES_Population_Make_New_Generation(population_t *parent_population, population_t *child_population);

double ES_Max_Ones_Fitness(genome_t *genome);
double ES_Rosenbrock(genome_t *genome);
double ES_Fitness_Function(genome_t *genome);

void ES_Population_Compute_Fitness(population_t *population);

#endif
