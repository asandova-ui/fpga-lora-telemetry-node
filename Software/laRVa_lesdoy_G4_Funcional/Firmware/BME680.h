
void startBME680();
void writeBME680(char data,char dir);
char readBME680(char dir);
int returnTemp();
int returnPressure();
int returnHumidity(int temp_comp);
