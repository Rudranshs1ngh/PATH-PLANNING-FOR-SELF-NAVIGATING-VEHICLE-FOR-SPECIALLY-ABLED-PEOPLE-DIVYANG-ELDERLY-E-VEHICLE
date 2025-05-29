#include <Servo.h>  //servo library

Servo myservo;      // create servo object to control servo

// Map size 6*6
#define row 6
#define col 6

int Echo = A4;  
int Trig = A5; 

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define carSpeed 100

const int STEPS_PER_REV = 200;
float duration, distance; 
int obstacle;

byte goalN = 35; // goal position on grid
byte openList[50]; // contains all the possible paths
byte closedList[50]; // contains the path taken
byte Path[50];
byte oLN=0, cLN=0;//the counters for the openList and closedList
byte curBotPos = 0 ; // holds current bot position
byte curBotDir = 1 ; // holds current bot facing direction(1 up  2 down 3 left  4 right)
byte curBotPos2;
byte curBotDir2;

struct Node
{
  byte g, h, f;
  byte parent;
  byte index;
  byte gridNom;
};

struct Grid
{
  Node Map[row][col];
} PF ;


byte H(byte curR, byte curC, byte goalS)  // manhattan distance heauristics function
{
 byte rowg, colg;
 byte manhattan=0;

 
   rowg = (byte)goalS/6;
   colg = goalS%6;
   manhattan += (abs(curR - rowg) + abs(curC - colg));
   
  return manhattan;
}


byte G(byte curR, byte curC)  // returns the number of gride have been traverd
{
  byte gValue, parInd;
  byte rowg, colg;
  parInd = PF.Map[curR][curC].parent;
 
  rowg = (byte)parInd/6;
  colg = parInd%6;
  gValue = PF.Map[rowg][colg].g;
  
  return (gValue+1);
}

byte FV(byte curG, byte curH) // the total "cost" of the path taken; adds H and G values for each tile
{
 byte fValue; 
  
  fValue = curG + curH;
  return fValue;
}

void setup(){ // sets up the program, builds the map, prints the grid for representation purposes, and takes user input for the goal
  myservo.attach(3,450,2400);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();

  myservo.write(90); //Put the ultrasonic in front of car
  
  buildMap();
  printGrid1();
  printGrid2();
  //setGoal();
  
}

// checks if the goal tile has been found
void loop(){ 
  
  if (!isGoal(curBotPos) && OLE)
  {
    _loop();                                      // the actual performance of the A* algorithm
  }
  else if (isGoal(curBotPos))
  {
    
    PathList();                                   // List the optimal path
    
    Serial.println("Path[i]");
    for(byte i=0;i<PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g;i++) {
    Serial.println(Path[i]);
    }
   delay(1000);
    while (1){
      
      movement(curBotPos,curBotDir);
      curBotPos = curBotPos2;
      curBotDir = curBotDir2;
        
        if (!isGoal(curBotPos)){
          break;      
        }
        
        Serial.println("Goal Reached");
        delay(100000);
     }   
  }  
}

void _loop(){                 // performs the A* algorithm, "main" program
  
  possMov(curBotPos);
  
  AddClosedList();
  
  printGrid2();
  
}

void buildMap() // builds the 6x6 map grid
{
  byte gridIn = 0;
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    PF.Map[i][j].gridNom = gridIn;
    PF.Map[i][j].index = 0;
    PF.Map[i][j].parent = 0;
    PF.Map[i][j].h = 0;
    PF.Map[i][j].g = 0;
    PF.Map[i][j].f = 0;
    
    gridIn++;    
   }
  }
}

void printGrid1()  // prints the grid, using indices 0 to 35 to represent the possible paths
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].gridNom);
    if (j != row-1)
    {
      if (PF.Map[i][j].gridNom < row-1)
      {
        Serial.print("  | ");
      }
      else
    Serial.print(" | ");
    } 
   }
  
  if (i != row-1)
    {
      Serial.println();
    Serial.print("----------------------------");
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void printGrid2() // prints the grid, 0 - untravelled | 1 - travelled | 2 - obstacles | 3 - goal
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].index);
    if (j != row-1)
    {
    Serial.print(" | ");
    } 
   }
  
  if (i != row-1)
    {
      Serial.println();
    Serial.print("----------------------");
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void setGoal() // asks user for input to set the goal state/tile
{
  byte goal;
  Serial.println("Where do you want to place your goal state?");
  Serial.println("Using the numbers displayaed in the earlier grid, enter a number to intialize as your goal state.");
  Serial.println();

  while (!Serial.available() )
  {
     goal = Serial.parseInt();
  }
 
  for (byte i = 0; i < row; i++)
  {
    for (byte k = 0; k < col; k++)
    {
      if (PF.Map[i][k].gridNom == goal)
      {
        
        PF.Map[i][k].index = 3;
        goalN = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 0)  /// initial start point 
      {
        PF.Map[i][k].index = 1;
        curBotPos = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 3 || PF.Map[i][k].gridNom == 14 || PF.Map[i][k].gridNom == 16 || PF.Map[i][k].gridNom == 18 || PF.Map[i][k].gridNom == 27 || PF.Map[i][k].gridNom == 29 || PF.Map[i][k].gridNom == 31)
      {
        PF.Map[i][k].index = 2;        // initial wall
      }
      else
      PF.Map[i][k].index = 0;          // initial free space
    }
  }
  printGrid2();
}
void possMov(byte gridNom) // checks the possible moves depending on the location of the current tile the bot is on
{
  byte rowp = (byte) gridNom / 6;
  byte colp = gridNom % 6;
  if (gridNom == 0) // checks the corner tiles | 2 possible moves
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom == 5)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom == 30)
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    }
  }
  else if (gridNom == 35)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 6);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 1);
    }   
  }
  else if (gridNom > 0 && gridNom < 5) // checks the tiles on the outermost edges of the map | 3 possible moves
  {
   if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }    
  }
  else if (gridNom%6==0)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom%6==5)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp- 1].index != 1 && PF.Map[rowp][colp- 1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom > 30 && gridNom < 35)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
  }
  else { // checks the remaining tiles | 4 possible moves
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
  }
     if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
  }
}

}

void AddOpenList(byte aol) // adds the potential possible moves to the openList
{
  
  openList[oLN++] = aol;
  heuristics(aol);
}

void heuristics(byte curIn) // calculates the "cost" of the tile
{
  byte hH, gH, fH;
  byte rowh = (byte) curIn / 6;
  byte colh = curIn % 6;

  hH = H(rowh, colh, goalN);
  PF.Map[rowh][colh].h = hH;
  gH = G(rowh, colh);
  PF.Map[rowh][colh].g = gH;
  fH = FV(hH,gH);
  PF.Map[rowh][colh].f = fH;
}

byte getNextFI() // returns the best heuristics value restricted by the current path the bot is taking
{
  byte rowf;
  byte colf;
  byte lowestF;
  byte lowest = openList[0];
  rowf = (byte) lowest / 6;
  colf = lowest % 6;
  lowestF = PF.Map[rowf][colf].f;
  
  for (byte i = 0; i < oLN; i++)
  {
    rowf = (byte) openList[i] / 6;
    colf = openList[i] % 6;
    
    if (PF.Map[rowf][colf].f <= lowestF) 
    {
      lowestF = PF.Map[rowf][colf].f;
      lowest = rowf*6 + colf;
    }
  }
  
  return lowest;
}

void AddClosedList() // adds the "best" tile to the closedList
{
  byte low = getNextFI(); 
  byte rowa, cola;

  closedList[cLN++] = low;
  rowa = (byte)low/6;
  cola = low%6;
  PF.Map[rowa][cola].index = 1;
  curBotPos = low;
  removeFOL(low); 
}

void PathList()  // List the optimal path
{
    for(byte i=1;i<PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g+1;i++){
      for(byte j=0;j<cLN;j++){
        if(PF.Map[closedList[j]/6][closedList[j]%6].g == i){
          Path[i-1]=closedList[j];
        }
      }
    }
}

void removeFOL(byte rfol) // removes previous potential paths from the openList, in order to get the "best" current path
{

  for (byte i = 0; i < oLN-30; i++)
  {
    if (openList[i] == rfol)
    {
      openList[i] = openList[i+1];
    }
    else
      openList[i] = openList[i+1];
  }
    oLN=oLN-1;
}

bool OLE() // checks if the openList is empty
{
  if (oLN == 0)
  {
    return true;
  }
  else
  return false;
}

bool isGoal(byte ig) // checks if the goal has been reached
{
  if (ig == goalN)
  {
    return true; 
  }
  else
  return false;
}

bool alreadyOnOL(byte rowaol, byte colaol) // checks if the tile is already on the openList
{
  byte indexol;
  bool on = false;

  indexol = rowaol*6 + colaol;
  for (byte i = 0; i < oLN; i++)
  {
    if (openList[i] == indexol)
    {
      on = true;
    }
  }
  
  return on;
}

byte movement(byte curBotPos,byte curBotDir) {
  
  curBotPos = PF.Map[Path[0]/6][Path[0]%6].parent;
  Serial.print("curBotPos_beforemovement:  ");
  Serial.println (curBotPos);
  Serial.print("curBotDir_beforemovement:  ");
  Serial.println (curBotDir);
  
  byte rowm, colm, parm;
  byte i = 0;

  while(!isGoal(curBotPos)){
     
      rowm = Path[i]/6;
      colm = Path[i]%6;
  
      if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 1){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 1){
        left();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 1){
        right();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 1){
        right();
        right();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;

      }

      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 2){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 2){
        right();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;

      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 2){
        left();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 2){
        right();
        right();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 3){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 3){
        right();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 3){
        left();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 3){
        right();
        right();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;      
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 4){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 4){
        right();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 4){
        left();
        curBotDir = 1;        
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 4){
        right();
        right();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
    }
  Serial.print("curBotPos_aftermovement: ");
  Serial.println (curBotPos); 
  Serial.print("curBotDir_aftermovement: ");
  Serial.println (curBotDir);
  curBotPos2 = curBotPos;
  curBotDir2 = curBotDir;
  return curBotPos2,curBotDir2;
  
  }

void rePathPlan(byte curBotPos,byte curBotDir) // re-design the path if encounter obstacles
{
 
  for (byte i = 0; i < 36; i++){

    if(PF.Map[i/6][i%6].index == 1){
      PF.Map[i/6][i%6].index = 0;
    }
    PF.Map[i/6][i%6].g = 0;
    PF.Map[i/6][i%6].h = 0;
    PF.Map[i/6][i%6].f = 0;
    PF.Map[i/6][i%6].parent = 0;
  }
   PF.Map[curBotPos/6][curBotPos%6].index = 1;
   PF.Map[goalN/6][goalN%6].index = 3;
  if(curBotDir == 1){
   PF.Map[(curBotPos + 6)/6][(curBotPos + 6)%6].index = 2;
  }
  else if(curBotDir == 2){
   PF.Map[(curBotPos - 6)/6][(curBotPos - 6)%6].index = 2;
  }
  else if(curBotDir == 3){
   PF.Map[(curBotPos + 1)/6][(curBotPos + 1)%6].index = 2;
  }
  else if(curBotDir == 4){
   PF.Map[(curBotPos - 1)/6][(curBotPos - 1)%6].index = 2;
  }
  
  oLN=0;
  cLN=0;

  for (byte i = 0; i<50; i++){
    openList[i] = 0; // contains all the possible paths
    closedList[i] = 0; // contains the path taken
    Path[i] = 0 ;   
  }
 Serial.print("curBotPos in re-path: ");
 Serial.println(curBotPos);
 Serial.print("curBotDir in re-path: ");
 Serial.println(curBotDir);
 printGrid2();
 
}
//Ultrasonic distance measurement Sub function
int check_obstacle(){
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW); 

  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;   

// Measure the response from the echo pin

// Determin distance from duration
// Use 343 metres per second as speed of sound
   if(Fdistance <= 30){
    obstacle = 1;
      Serial.println("Obstacle detected!");
   }
   else{
    obstacle = 0;
   }
   return obstacle;
}
// Function for forward
void forward(){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("forward");
  delay(1000);
  stop();
  delay(1000);
}

void back() {
  right();
  right();
}

void left() {
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
  delay(600);
  stop();
}

void right() {
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
  delay(650);
  stop();
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 
