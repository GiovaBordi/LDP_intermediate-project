/*
	Variabili:
		static constexpr int BUFFER_DIM {10};
		vector<double>* secia[BUFFER_DIM];
		int coa;
		int dimension;
*/

#include "../include/LidarDriver.h"

namespace lidar_driver {
	// Costruttore provvisorio
	LidarDriver::LidarDriver() : elPiNovo{0}, elPiVecio{0}, dimension{0}, secia{new std::vector<double>[DIM_LETTURE]} {
		// DA IMPLEMENTARE
	}

	LidarDriver::LidarDriver(const LidarDriver &) {
		// DA IMPLEMENTARE
	}

	/* Funzione new_scan(vector<double> v):
		1. Il vettore passato v, viene controllato, si verifica non punti ad un elemento nullo;
		2. Se la dimensione del vettore fornito non è quella richiesta (pari alla costante DIM_LETTURE), il vettore viene modificato con la funzione resize di std::vector;
		3. Superati i controlli precedenti, il vettore viene inserito nel buffer;
		4. Vengono opportunamente incrementati gli indici e il numero di vettori salvati.

	*/
	void LidarDriver::new_scan(std::vector<double> v) {
		// Verifica se il vettore fornito è un puntatore nullo
		if (&v == nullptr)
			throw NullVettorError();

		// Si verifica se il vettore fornito rispetta le condizioni nel numero di elementi, nel caso il vettore viene modificato dalla funzione resize di std::vector;
		if (v.size() != DIM_LETTURE)
			v.resize(DIM_LETTURE, 0);

		// INSERIMENTO VETTORE
		// elPiNovo punta all'ultimo elemento inserito, bisogna dunque farlo avanzare tranne nel caso in cui dimension = 0, in tal caso è sufficiente conservare l'indice attuale.
		// e procedere con l'inserimento. Occorre prevedere il caso in cui, incrementando, l'indice elPiNovo giunga al termine del buffer, in questo caso viene azzerato
		// per ricominciare gli inserimenti dall'inizio del buffer.
		elPiNovo = (dimension == 0) ? elPiNovo : (elPiNovo + 1) % BUFFER_DIM;
		secia[elPiNovo] = v;

		// Ora vanno incrementati l'indice dell'elemento più vecchio e la variabile grandezza:
		//  - L'indice all'elemento più vecchio non viene alterato se il buffer non è pieno, nel caso il buffer risulti pieno la funzione new_scan() comincia a sovrascrivere
		//    gli elementi più vecchi, è quindi necessario incrementare l'indice elPiVecio;
		//  - La  grandezza si incrementa fino ad arrivare al riempimento del buffer, in quel caso il valore rimane stabile.
		elPiVecio = (dimension == BUFFER_DIM) ? (elPiVecio + 1) % BUFFER_DIM : elPiVecio;
		dimension = (dimension == BUFFER_DIM) ? BUFFER_DIM : dimension + 1;
	}

	/* Funzione get_scan():
		1. Viene verificato se il buffer è vuoto, nel caso viene lanciata l'eccezione "NoGheSonVettoriError" ("NonCiSonoVettoriError");
		2. Superato il controllo, la dimensione del buffer viene decrementata;
		3. Nella variabile "scoase" ("immondizia") viene salvato l'indice "elPiVecio", che viene modificato per puntare al successivo elemento presente da più tempo nel buffer;
		4. Il vettore rimosso viene quindi restiuito all'utente.
	*/
	std::vector<double> LidarDriver::get_scan() {
		// Si verifica se ci sono vettori di misurazioni, in caso contrario viene lanciata l'eccezione "NoGheSonVettoriError".
		if (dimension == 0)
			throw NoGheSonVettoriError();

		// Se il vettore non è vuoto, contiene almeno un elemento, si può quindi decrementare la variabile dimension:
		dimension--;

		// L'elemento più vecchio, e quindi quello da eliminare, è puntato dalla variabile elPiVecio. Risulta necessario decrementare la variabile prima di ritornare.
		// Salvo l'indice attuale in un'opportuna variabile, in modo da procedere poi con il ritorno del vettore di interesse.
		int scoase = elPiVecio;
		elPiVecio = (dimension != 0) ? (elPiVecio + 1) % BUFFER_DIM : elPiVecio;
		return secia[scoase];
	}

	/* Funzione clear_buffer():
		Libera il buffer dalle misurazioni memorizzate e reimposta gli indici di posizione
	 */
	void LidarDriver::clear_buffer() {
		// Se il buffer è vuoto lancia l'eccezione "NoGheSonVettoriError" ("NonCiSonoVettoriError");
		if (secia == nullptr)
			return;
		
		// Altrimenti copia il riferimento di secia
		std::vector<double> *t{secia};

		// Assegna a secia un nuovo vettore
		secia = new std::vector<double>[DIM_LETTURE];
		
		// Libera la memoria occupata dai dati vecchi memorizzati in secia
		delete[] t;

		// Reimposta le variabili dell'oggetto
		elPiNovo = elPiVecio = dimension = 0;
	}

	double LidarDriver::get_distance(double) {
		// DA IMPLEMENTARE
		return 0;
	}
}
