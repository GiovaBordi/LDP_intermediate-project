# LDP-intermediate-project
Progetto intermedio del corso di Laboratorio di Programmazione

## TO-DO List
1. gestire la chiamata del distruttore dei vettori sovrascritti nella funzione new_scan: quando il buffer è pieno si ha che il vettore più vecchio viene sovrascritto da quello nuovo appena inserito.
2. capire come avviene l'accesso ai vari vettori di scansioni e alla loro posizione in memoria: se creo un vettore di scansioni, lo inserisco nel buffer e poi lo distruggo, posso ancora accederci dal buffer?
3. avrebbe senso usare un semplice vettore C con puntatori a vettori di double al posto di avere un puntatore a vettore di puntatori a vettori di double?

## Descrizione Classe LidarDriver
La classe memorizza un certo numero di scansioni, effettuate da un Lidar, in un buffer. Le scansioni sono ottenute dalla misurazione della distanza per vari angoli consecutivi da 0° a 180°. La risoluzione angolare, ovvero la distanza tra due misurazioni consecutive, dipende dal tipo di Lidar ed è compresa tra 0.1° e 1°.

## Specifiche classe LidarDriver
- riceve in input la risoluzione angolare del Lidar
- memorizza le scansioni in un buffer di dimensione costante BUFFER_DIM
- se il buffer è pieno, la nuova scansione sovrascrive quella più vecchia
- le scansioni sono std::vector<double>

## Dettagli implementativi
| Variabili private | descrizione |
| - | - |
| secia | std::vector<*std::vector<double>> vettore di puntatori a vettori double, ovvero vettore di puntatori alle varie scansioni, con dimensione fissata di BUFFER_DIM |
| elPiNovo | indice dell'ultimo elemento inserito |
| elPiVecio | indice del primo elemento inserito |
| dimension | numero di elementi nel buffer |
| resolusion | risoluzione angolare dello strumento |

| Costanti | descrizione |
| - | - |
| BUFFER_DIM | dimensione massima del buffer |
| MIN_ANGLE | angolo minimo scansione |
| MAX_ANGLE | angolo massimo scansione |
| MIN_RESOLUTION | risoluzione minima |
| MAX_RESOLUTION | risoluzione massima |

| Classi eccezioni | descrizione |
| - | - |
| NoGheSonVettoriError | eccezione lanciata nel caso di operazioni invalide su buffer vuoto |
| NullVettorError | eccezione lanciata quando si ricevono parametri nullptr/null |
| ResolusionForaDaiRangeError | eccezione lanciata quando la risoluzione angolare è al di fuori dei range specificati nella consegna |

| Costruttori | descrizione |
| - | - |
| LidarDriver(double) | specifica la risoluzione angolare dello strumento |
| LidarDriver(const LidarDriver &) | costruttore di copia |
| LidarDriver(LidarDriver &&) | costruttore di move |
| ~LidarDriver() | distruttore |

| Member function pubbliche | descrizione |
| - | - |
| void new_scan(vector<double>) | aggiunge una nuova scansione al buffer |
| vector<double> get_scan() | restituisce la più vecchia scansione inserita e la rimuove dal buffer |
| void clear_buffer() | elimina tutte le scansioni nel buffer |
| double get_distance(double) const | restituisce la lettura corrispondente all'angolo specificato come parametro (o al suo angolo più vicino) presente nell'ultima scansione inserita nel buffer senza rimuoverla |
| operator<< const | stampa l'ultima scansione inserita nel buffer senza rimuoverla |
