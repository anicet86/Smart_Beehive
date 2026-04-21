/*
 * goertzel.c
 *
 *  Created on: Jan 11, 2026
 *      Author: kemazhu
 */

#include "goertzel.h"
#include <math.h>

float goertzel_process(const int16_t *samples, uint16_t num_samples)
{
    if (samples == NULL || num_samples == 0) return 0.0f;

    // Calcul du coefficient de Goertzel
    float k = 0.5f + ((float)num_samples * TARGET_FREQ_HZ) / SAMPLE_RATE_HZ;
    float w = (2.0f * M_PI * k) / (float)num_samples;
    float cos_w = cosf(w);
    float coeff = 2.0f * cos_w;

    // Variables d'état
    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;

    // Boucle de traitement
    for (uint16_t i = 0; i < num_samples; i++)
    {
        q0 = (float)samples[i] + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
    }

    // Calcul de l'énergie
    float real = (q1 - q2 * cos_w);
    float imag = (q2 * sinf(w));
    float magnitude_squared = real * real + imag * imag;

    return magnitude_squared; // Énergie (pas besoin de sqrt pour détection)
}
