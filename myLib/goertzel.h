/*
 * goertzel.h
 *
 *  Created on: Jan 11, 2026
 *      Author: kemazhu
 */

#ifndef GOERTZEL_H_
#define GOERTZEL_H_

#include <stdint.h>

// Configuration
#define TARGET_FREQ_HZ      250.0f    // Fréquence cible (bourdonnement abeille)
#define SAMPLE_RATE_HZ      8000.0f   // Fréquence d'échantillonnage (doit être >= 2*TARGET)
#define BLOCK_SIZE          128       // Nombre d'échantillons par calcul

// Fonction principale
float goertzel_process(const int16_t *samples, uint16_t num_samples);

#endif /* GOERTZEL_H_ */
