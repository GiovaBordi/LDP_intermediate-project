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
	
	Osservazioni (5.0 - G. Simonetto)
		1. correzione sull'implementazione in memoria del buffer
		2. aggiunta funzione get_last per implementazione dell'overloading operator<<

	Osservazioni (6.0 - G. Simonetto)
		1. non serve il costruttore e l'operatore di copia perché è sufficiente la shallow copy membro a membro
		   siccome non abbiamo puntatori da gestire, il vettore secia applica la copia membro a membro sugli
		   elementi che contiene e, non contenendo puntatori, è tutto ok
		2. serve, invece, il costruttore di move e l'operatore di move per risparmiare dati e tempo
*/

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <ostream>
#include <vector>

namespace lidar_driver {
	class LidarDriver {
		public:
			// costruttori e distruttori
			LidarDriver(double);
			LidarDriver(LidarDriver &&);

			// member function
			void new_scan(std::vector<double>);
			std::vector<double> get_scan();
			std::vector<double> get_last() const;
			void clear_buffer();
			double get_distance(double) const;

			// overloading operatori
			void operator=(LidarDriver &&);

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
