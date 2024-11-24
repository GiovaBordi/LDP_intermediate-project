/*
	FILE HEADER LIDARDRIVER.H
	Versione:	4.0
	Data:		24/11/2024
	Autore:		Giovanni Bordignon - Giacomo Simonetto

	Osservazioni (1.0 - G. Bordignon):
		1. BUFFER_DIM dichiarata costante e statica. OK?
		2. Indice "coa", da trovare nome per eventuale secondo indice.
		3. Per ora dichiarato solo il costruttore di default e copia, serve altro?.
		4. Valutare se tornare il vettore sovrascritto dalla funzione new_scan();
		5. (Funzioni const)

	Osservazioni (3.0 - G. Bordignon)
		1. Verificare i punti 1 (IMPORTANTE, valida anche per MAX_LETTURE) - 3 - 4 delle Osservazioni della versione 1.0;
		2. Valutare eventuale cambio nome degli indici agli elementi.

	Osservazioni (4.0 -  G. Simonetto)
		1. aggiunta del namespace
		2. aggiunti qualche commenti nella classe
		3. correzione impaginazione (usiamo le tab e non i 4 spazi)
		4. implementazione costruttori e distruttori
		5. modifica variabili e costanti per maggiore flessibilità sulla risoluzione angolare:
			- aggiunta resolusion
			- aggiunta costante MIN_ANGLE
			- aggiunta costante MAX_ANGLE
			- aggiunta costante MIN_RESOLUTION
			- aggiunta costante MAX_RESOLUTION
			- sostituzione costante DIM_LETTURE con dimScansioni
			- correzione dichiarazione di secia -> è puntatore a vettore di puntatori a vettori di double
*/

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <iostream>
#include <vector>

namespace lidar_driver {
	class LidarDriver {
		public:
			LidarDriver();
			LidarDriver(const LidarDriver &);
			void new_scan(std::vector<double>);
			std::vector<double> get_scan();
			void clear_buffer();
			double get_distance(double);

			class NoGheSonVettoriError {}; // Eccezione "NoGheSonVettori" ("NoCiSonoVettori")
			class NullVettorError {};
		private:
			static constexpr int BUFFER_DIM{10};
			static constexpr int DIM_LETTURE{181};
			std::vector<double> *secia; // BUFFER ("secia" = secchio)
			int elPiNovo;   // Indice all'ultimo vettore inserito ("elPiNovo" = ilPiùNuovo)
			int elPiVecio;  // Indice al vettore da più tempo presente nel buffer ("elPiVecio" = ilPiùVecchio)
			int dimension;  // Dimensione attuale del buffer
	};

	std::ostream &operator<<(std::ostream &, LidarDriver &);
};

#endif // LIDARDRIVER_H