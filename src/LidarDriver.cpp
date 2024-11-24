/*
	Variabili:
		static constexpr int BUFFER_DIM {10};
		vector<double>* secia[BUFFER_DIM];
		int coa;
		int dimension;
*/

#include "../include/LidarDriver.h"
#include <cmath>

namespace lidar_driver {
	/* Costruttore con risoluzione:
		1. riceve come parametro la risoluzione dello strumento e verifica che sia valida
		2. imposta le variabili ai valori iniziali
		3. alloca la dimensione del buffer necessaria
		
		NB: el numero dee letture per scansione xe (MAX_ANGLE-MIN_ANGLE) / resolusion + 1
	*/
	LidarDriver::LidarDriver(double resolusion) {
		// verifica che la risoluzione sia valida
		if (resolusion < MIN_RESOLUTION || resolusion > MAX_RESOLUTION)
			throw ResolusionForaDaiRangeError();
		
		// inizializza le variabili ai valori di default
		elPiNovo = elPiVecio = dimension = 0;
		this->resolusion = resolusion;
		dimScansioni = (MAX_ANGLE - MIN_ANGLE) / resolusion + 1;

		// alloca il buffer dinamicamente
		secia = new std::vector<std::vector<double>*>[BUFFER_DIM];
	}

	/* Costruttore di copia:
		1. riceve come parametro un oggetto da copiare
		2. esegue la deep copy dell'oggetto
	*/
	LidarDriver::LidarDriver(const LidarDriver &ld) {
		// inizializzazione variabili con i valori dell'oggetto da copiare
		elPiNovo = ld.elPiNovo;
		elPiVecio = ld.elPiVecio;
		dimension = ld.dimension;
		resolusion = ld.resolusion;

		// creo un nuovo buffer e ci copio i valori da quello dell'oggetto da copiare
		secia = new std::vector<std::vector<double>*>[BUFFER_DIM];
		std::copy(ld.secia, ld.secia + BUFFER_DIM, secia);
	}

	/* Costruttore di move:
		1. riceve come parametro un oggetto da "smembrare"
		2. copia le variabili da copiare e sposta i riferimenti da spostare
	*/
	LidarDriver::LidarDriver(LidarDriver &&ld) {
		// inizializzazione variabili con i valori dell'oggetto da smembrare
		elPiNovo = ld.elPiNovo;
		elPiVecio = ld.elPiVecio;
		dimension = ld.dimension;
		resolusion = ld.resolusion;

		// svoto a secia ne'e scoasse
		std::vector<std::vector<double> *> *scoasse = secia;

		// riempio a secia co e' nove robe
		secia = ld.secia;
		ld.secia = nullptr;

		// riciclo e' scoasse
		delete[] scoasse;
	}
	
	/* Distruttore:
		1. Dealloca la memoria allocata dinamicamente
	*/
	LidarDriver::~LidarDriver() {
		delete[] secia;
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
		if (v.size() != dimScansioni)
			v.resize(dimScansioni, 0);

		// INSERIMENTO VETTORE
		// elPiNovo punta all'ultimo elemento inserito, bisogna dunque farlo avanzare tranne nel caso in cui dimension = 0, in tal caso è sufficiente conservare l'indice attuale.
		// e procedere con l'inserimento. Occorre prevedere il caso in cui, incrementando, l'indice elPiNovo giunga al termine del buffer, in questo caso viene azzerato
		// per ricominciare gli inserimenti dall'inizio del buffer.
		// Per accere al posto dove salvare il nuovo vettore: dereferenzio secia per accedere al vettore di buffer (*secia).. e mi sposto all'elPiNovo-esimo elemento ..[elPiNovo]..
		// dove ci salvo l'indirizzo di v .. = &v;
		elPiNovo = (dimension == 0) ? elPiNovo : (elPiNovo + 1) % BUFFER_DIM;
		(*secia)[elPiNovo] = &v;

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
		return *((*secia)[scoase]);
	}

	/* Funzione clear_buffer():
		Libera il buffer dalle misurazioni memorizzate e reimposta gli indici di posizione
	 */
	void LidarDriver::clear_buffer() {
		// Se il buffer è vuoto non c'è da fare nulla
		if (dimension == 0)
			return;
		
		// Altrimenti copia il riferimento di secia
		std::vector<std::vector<double>*> *t{secia};

		// Assegna a secia un nuovo vettore
		secia = new std::vector<std::vector<double>*>[BUFFER_DIM];
		
		// Libera la memoria occupata dai dati vecchi memorizzati in secia
		delete[] t;

		// Reimposta le variabili dell'oggetto
		elPiNovo = elPiVecio = dimension = 0;
	}

	/* Funzione get_distance(double):
		1. controlla che esista il valore da restituire: buffer non vuoto e angolo valido
		2. trova l'indice dell'elemento cercato
		3. restituisce il valore cercato
	*/
	double LidarDriver::get_distance(double angolo) const {
		// fa gli eventuali controlli necessari
		if (dimension == 0)
			throw NoGheSonVettoriError();
		if (angolo < MIN_ANGLE || angolo > MAX_ANGLE)
			throw AngoloForaDaiRangeError();
		
		// individua l'indice secondo la seguente proporzione
		/**
		 * l'angolo dell'i-esimo elemento con i in [0, dimScansioni-1] è dato da: angolo = indice * resolusion
		 *   es: indice = 10   resolusion = 1    -> angolo = 10*1 = 10
		 *       indice = 10   resolusion = 0.5  -> angolo = 10*0.5 = 5
		 *       indice = 0    resolusion = ?    -> angolo = ?*0 = 0
		 *       indice = 180  resolusion = 1    -> angolo = 180*1 = 180
		 *       indice = 360  resolusion = 0.5  -> angolo = 360*0.5 = 180
		 * 
		 * per passare dall'angolo all'indice devo fare la formula inversa: indice = angolo / resolusion
		 *   es: angolo = 10   resolusion = 1    -> indice = 10/1 = 10
		 *       angolo = 5    resolusion = 0.5  -> indice = 5/0.5 = 10
		 *       angolo = 0    resolusion = ?    -> indice = 0/? = 0
		 *       angolo = 180  resolusion = 1    -> indice = 180/1 = 180
		 *       angolo = 180  resolusion = 0.5  -> indice = 180/0.5 = 360
		 * 
		 * osservazioni per arrotondamenti nel caso di una risoluzione angolare non intera
		 *   resolusion = 0.5
		 *   indice:  0  -  1  -  2  -  3  -  4  -  5  -  6  -  7  -  8  - ...
		 *   angolo: 0.0 - 0.5 - 1.0 - 1.5 - 2.0 - 2.5 - 3.0 - 3.5 - 4.0 - ...
		 *   se scelgo un angolo di 0.6  -> indice = 0.6/0.5 = 1.2 -> 1
		 *                       di 0.7  -> indice = 0.7/0.5 = 1.4 -> 1
		 *                       di 0.75 -> indice = 0.75/0.5 = 1.5 -> 2*
		 *                       di 0.8  -> indice = 0.8/0.5 = 1.6 -> 2*
		 * alla fine bisogna aggiungere un arrotondamento all'intero più vicino per eseguire correttamente la divisione
		 * per fare ciò usiamo la funzione double std::round(double) e un cast esplicito da double a int
		 */
		int index = static_cast<int>(std::round(angolo / resolusion));

		// restituisce quanto cercato
		return (*(*secia)[elPiNovo])[index];
	}
}
