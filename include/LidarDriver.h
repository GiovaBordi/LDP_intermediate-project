/*
	FILE HEADER LIDARDRIVER.H
	Versione:	4.0
	Data:		24/11/2024
	Autore:		Giovanni Bordignon - Giacomo Simonetto

	Osservazioni (1.0 - G. Bordignon):
		1. Dichiarazioni costanti statiche (BUFFER_DIM).
		2. Dichiarazione primo indice (coa), in valutazione nome del secondo indice.
		3. Dichiarazione dei costruttori di default e copia.
		4. Dichiarazioni di alcune funzioni membro.

	Osservazioni (3.0 - G. Bordignon)
		1. Modifiche varie, alla luce anche delle osservazioni della v. 1.0.
		2. Valutato cambio nomi agli indici.

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
	
	Osservazioni (5.0 - G. Simonetto)
		1. correzione sull'implementazione in memoria del buffer
		2. aggiunta funzione get_last per implementazione dell'overloading operator<<
*/

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <iostream>
#include <vector>

namespace lidar_driver {
	class LidarDriver {
		public:
			// costruttori e distruttori
			LidarDriver(double);
			LidarDriver(const LidarDriver &);
			LidarDriver(LidarDriver &&);

			// member function
			void new_scan(std::vector<double>);
			std::vector<double> get_scan();
			std::vector<double> get_last() const;
			void clear_buffer();
			double get_distance(double) const;

			// classi per lancio di errori
			class NoGheSonVettoriError{}; // Eccezione "NoGheSonVettori" ("NoCiSonoVettori")
			class NullVettorError{};
			class ResolusionForaDaiRangeError{};
			class AngoloForaDaiRangeError{};

		private:
			// costanti private
			static constexpr int BUFFER_DIM{10};
			static constexpr int MIN_ANGLE{0};
			static constexpr int MAX_ANGLE{180};
			static constexpr double MIN_RESOLUTION{0.1};
			static constexpr double MAX_RESOLUTION{1};

			// variabili private
			std::vector<std::vector<double>> secia;	// BUFFER ("secia" = secchio)
			int elPiNovo;		// Indice all'ultimo vettore inserito ("elPiNovo" = ilPiùNuovo)
			int elPiVecio;		// Indice al vettore da più tempo presente nel buffer ("elPiVecio" = ilPiùVecchio)
			int dimension;		// Dimensione utilizzata del buffer
			int dimScansioni;	// Dimensione dei vettori delle scansioni
			double resolusion;	// Risoluzione angolare dello strumento
	};

	// overloading operatore output
	std::ostream &operator<<(std::ostream &, const LidarDriver &);
}

#endif // LIDARDRIVER_H
