/*
    Variabili:
        static constexpr int BUFFER_DIM {10};
        vector<double>* secia[BUFFER_DIM];
        int coa;
        int dimension;
*/

LidarDriver::LidarDriver(){

}

LidarDriver::LidarDriver(const LidarDriver&){

}

void LidarDriver::new_scan(std::vector<double>){
  
}

std::vector<double> LidarDriver::get_scan(){
  
}

void LidarDriver::clear_buffer(){
  for(int i=0;i<10;i++){
			if(this->secia == nullptr)
				continue;
			std::vector<double>* t {this->secia};
			this->secia = nullptr;
			delete[] t;
			this->coa = 0;
		}
		return;
}

double LidarDriver::get_distance(double){
  
}
