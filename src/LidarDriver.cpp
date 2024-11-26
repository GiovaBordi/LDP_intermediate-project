#include "../include/LidarDriver.h"
#include <cmath>  // per std::round nella funzione get_distance
#include <string> // per overloading operator<<

namespace lidar_driver {
	/* Costruttore con risoluzione:
		1. riceve come parametro la risoluzione dello strumento e verifica che sia valida
		2. imposta le variabili ai valori iniziali
		3. ridimensiona il buffer alla dimensione necessaria
		
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

		// ridimensiona il buffer alla dimensione necessaria
		secia.resize(BUFFER_DIM);
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

		// copio i valori del buffer dell'oggetto da copiare
		secia = ld.secia;
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

		// la funzione swap scambia i riferimenti dei dati tra i due vettori
		secia.swap(ld.secia);
	}

	/* Funzione new_scan(vector<double> v):
		1. ricevo il vettore come copia per due motivi:
			- non vogliamo alterare il vettore della funzione chiamante perché non si sa mai cosa potrebbe voler fare l'utente dopo averlo inserito nel buffer
			- vogliamo tenere distinti il vettore della funzione chiamante e quello nel buffer per evitare che ad un certo punto non si possa più accedere al
			  vettore perché è stato invocato il suo distruttore per qualsiasi motivo
			- sappiamo che passando il vettore per reference (non const) è possibile effettuare l'inserimento in maniera molto efficiente con 0 copie, ma
			  ci sembrava più corretto e user-proof farlo in questo modo
		2. se la dimensione del vettore fornito non è quella richiesta, il vettore viene modificato con la funzione resize come richiesto
		3. il vettore viene inserito nel buffer con la funzione swap:
			- il vettore da inserire e il vettore nella secia si scambiano i riferimenti dei dati per cui 0 copie :)
			- il vettore nella secia conterrà i dati della scansione da inserire e il vettore-parametro conterrà i vecchi dati del vettore nella secia 
			- al termine della funzione viene invocato il distruttore sul vettore-parametro, così vengono eliminati i vecchi dati sovrascritti, per cui 0 memory leak :)
		4. vengono opportunamente incrementati gli indici e il numero di vettori salvati
	*/
	void LidarDriver::new_scan(std::vector<double> v) {
		// Si verifica se il vettore fornito rispetta le condizioni nel numero di elementi, nel caso il vettore viene modificato dalla funzione resize di std::vector;
		if (v.size() != dimScansioni)
			v.resize(dimScansioni, 0);

		// INSERIMENTO VETTORE
		// elPiNovo punta all'ultimo elemento inserito, bisogna dunque farlo avanzare tranne nel caso in cui dimension = 0, in tal caso è sufficiente conservare l'indice attuale.
		// e procedere con l'inserimento. Occorre prevedere il caso in cui, incrementando, l'indice elPiNovo giunga al termine del buffer, in questo caso viene azzerato
		// per ricominciare gli inserimenti dall'inizio del buffer.
		// Per inserire il nuovo vettore, uso la funzione swap
		elPiNovo = (dimension == 0) ? elPiNovo : (elPiNovo + 1) % BUFFER_DIM;
		secia[elPiNovo].swap(v);

		// Ora vanno incrementati l'indice dell'elemento più vecchio e la variabile dimension:
		//  - L'indice all'elemento più vecchio non viene alterato se il buffer non è pieno, nel caso il buffer risulti pieno la funzione new_scan() comincia a sovrascrivere
		//    gli elementi più vecchi, è quindi necessario incrementare l'indice elPiVecio;
		//  - La dimension si incrementa fino ad arrivare al riempimento del buffer, in quel caso il valore rimane stabile.
		elPiVecio = (dimension == BUFFER_DIM) ? (elPiVecio + 1) % BUFFER_DIM : elPiVecio;
		dimension = (dimension == BUFFER_DIM) ? BUFFER_DIM : dimension + 1;
	}

	/* Funzione get_scan():
		1. viene verificato se il buffer è vuoto, nel caso viene lanciata l'eccezione "NoGheSonVettoriError" ("NonCiSonoVettoriError");
		2. superato il controllo, la dimensione del buffer viene decrementata;
		3. nella variabile "scoase" ("immondizia") viene salvato l'indice "elPiVecio", che viene modificato per puntare al successivo elemento presente da più tempo nel buffer;
		4. il vettore rimosso viene quindi restiuito all'utente.
		Osservazione: il vettore non viene effettivamente rimosso dal buffer, ma si "marca" la cella come libera per un'eventuale nuovo inserimento
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

	/* Funzione get_last():
        Autore: G. Bordignon
        La funzione restituisce l'ultimo vettore inserito, in caso il buffer sia vuoto viene lanciata l'eccezione "NoGheSonVettoriError".

		La funzione non è richiesta dalle specifiche, ma viene usata dalla helper function dell'overloading dell'operatore <<
    */
    std::vector<double> LidarDriver::get_last() const {
        if (dimension == 0)
            throw NoGheSonVettoriError();
        
        return secia[elPiNovo];
    }

	/* Funzione clear_buffer():
		1. se il buffer è vuoto, non faccio nulla, perché no xe poe svotàr na secia già vota
		2. reimposta gli indici di posizione e la dimensione
		3. svuota tutti i vettori così si occupa meno memoria
	 */
	void LidarDriver::clear_buffer() {
		// Se il buffer è vuoto non c'è da fare nulla
		if (dimension == 0)
			return;
		
		// Reimposta le variabili dell'oggetto
		elPiNovo = elPiVecio = dimension = 0;

		// Svuota tutti i vettori -> effettuo una swap con vettore vuoto
		for (int i = 0; i < BUFFER_DIM; i++) {
			std::vector<double>().swap(secia[i]);
		}
	}

	/* Funzione get_distance(double):
		1. controlla che esista il valore da restituire: buffer non vuoto e angolo valido
		2. trova l'indice dell'elemento cercato
		3. restituisce il valore cercato

		Osservazioni sul calcolo dell'indice e sulla conversione angolo->indice:
		1.  l'angolo dell'i-esimo elemento con i in [0, dimScansioni-1] è dato da: angolo = indice * resolusion
			es: indice = 10   resolusion = 1    -> angolo = 10*1 = 10
				indice = 10   resolusion = 0.5  -> angolo = 10*0.5 = 5
				indice = 0    resolusion = ?    -> angolo = ?*0 = 0
				indice = 180  resolusion = 1    -> angolo = 180*1 = 180
				indice = 360  resolusion = 0.5  -> angolo = 360*0.5 = 180

		2.  per passare dall'angolo all'indice devo fare la formula inversa: indice = angolo / resolusion
			es: angolo = 10   resolusion = 1    -> indice = 10/1 = 10
				angolo = 5    resolusion = 0.5  -> indice = 5/0.5 = 10
				angolo = 0    resolusion = ?    -> indice = 0/? = 0
				angolo = 180  resolusion = 1    -> indice = 180/1 = 180
				angolo = 180  resolusion = 0.5  -> indice = 180/0.5 = 360

		3.  nel caso di arrotondamenti dovuti ad una non perfetta coincidenza tra angolo e indice (es. con resolusion = 0.5)
			indice:  0  -  1  -  2  -  3  -  4  -  5  -  6  -  7  -  8  - ...
			angolo: 0.0 - 0.5 - 1.0 - 1.5 - 2.0 - 2.5 - 3.0 - 3.5 - 4.0 - ...
			se scelgo un angolo di 0.6  -> indice = 0.6/0.5 = 1.2 -> 1
								di 0.7  -> indice = 0.7/0.5 = 1.4 -> 1
								di 0.75 -> indice = 0.75/0.5 = 1.5 -> 2*
								di 0.8  -> indice = 0.8/0.5 = 1.6 -> 2*
			alla fine, quindi, bisogna aggiungere un arrotondamento all'intero più vicino per eseguire correttamente la divisione
			per fare ciò usiamo la funzione double std::round(double) e un cast esplicito da double a int
	 */
	double LidarDriver::get_distance(double angolo) const {
		// fa gli eventuali controlli necessari
		if (dimension == 0)
			throw NoGheSonVettoriError();
		if (angolo < MIN_ANGLE || angolo > MAX_ANGLE)
			throw AngoloForaDaiRangeError();
		
		// conversione angolo -> indice come descritto sopra
		int index = static_cast<int>(std::round(angolo / resolusion));

		// restituisce quanto cercato
		return secia[elPiNovo][index];
	}

	/* Ridefinizione dell'operatore <<
        Autore: G. Bordignon
        Con un try - catch viene gestito il caso in cui il buffer sia vuoto: la funzione get_last lancia infatti l'eccezione "NoGheSonVettori", che, recepita dalla presente
        funzione, permette di inviare immediatamente allo stream di output una generica stampa di un array vuoto. Nel caso l'eccezione non si presenti, sintomo che il buffer
        non è vuoto, l'ultimo vettore inserito viene restituito dalla funzione get_last() e stampato con un'opportuna formattazione.
    */
    std::ostream &operator<<(std::ostream& os, const LidarDriver& ld) {
        try {
			std::vector<double> temp = ld.get_last();
			std::string s = "{ ";
			for (int i = 0; i < temp.size(); i++) {
				s += std::to_string(temp[i]);
				if (i < temp.size() - 1)
					s += ", ";
			}
			s += " }\n";

			return os << s;
        }
        catch (LidarDriver::NoGheSonVettoriError) {
            return os << "{ }\n";
        }
    }
}
