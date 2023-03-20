package frc.robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TargetSelector implements Loggable {
    //indacate which target is selected for automatic aiming
   
    static boolean row[] = new boolean[3];
    static boolean column[] = new boolean[3];
    static boolean grid[] = new boolean[3];

    static  boolean target[][][] = new boolean[3][3][3];

    

    final static String tabname = "Target Selector";

    @Log.BooleanBox(rowIndex = 0, columnIndex = 0, tabName = tabname, width = 1, height = 1 )
    static boolean backleftA = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 1, tabName = tabname, width = 1, height = 1 )
   static  boolean backcenterA = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 2, tabName = tabname, width = 1, height = 1 )
   static  boolean backrightA = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 0, tabName = tabname, width = 1, height = 1 )
   static  boolean midleftA = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 1, tabName = tabname, width = 1, height = 1 )
   static  boolean midcenterA = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 2, tabName = tabname, width = 1, height = 1 )
   static  boolean midrightA = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 0, tabName = tabname, width = 1, height = 1 )
   static  boolean frontleftA = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 1, tabName = tabname, width = 1, height = 1 )
   static  boolean frontcenterA = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 2, tabName = tabname, width = 1, height = 1 )
   static  boolean frontrightA = false;
  
    @Log.BooleanBox(rowIndex = 0, columnIndex = 4, tabName = tabname, width = 1, height = 1 )
   static  boolean backleftB = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 5, tabName = tabname, width = 1, height = 1 )
   static  boolean backcenterB = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 6, tabName = tabname, width = 1, height = 1 )
   static  boolean backrightB = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 4, tabName = tabname, width = 1, height = 1 )
   static  boolean midleftB = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 5, tabName = tabname, width = 1, height = 1 )
   static  boolean midcenterB = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 6, tabName = tabname, width = 1, height = 1 )
   static  boolean midrightB = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 4, tabName = tabname, width = 1, height = 1 )
   static  boolean frontleftB = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 5, tabName = tabname, width = 1, height = 1 )
   static  boolean frontcenterB = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 6, tabName = tabname, width = 1, height = 1 )
   static  boolean frontrightB = false;
  
    @Log.BooleanBox(rowIndex = 0, columnIndex = 8, tabName = tabname, width = 1, height = 1 )
   static  boolean backleftC = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 9, tabName = tabname, width = 1, height = 1 )
   static  boolean backcenterC = false;
    @Log.BooleanBox(rowIndex = 0, columnIndex = 10, tabName = tabname, width = 1, height = 1 )
   static  boolean backrightC = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 8, tabName = tabname, width = 1, height = 1 )
   static  boolean midleftC = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 9, tabName = tabname, width = 1, height = 1 )
   static  boolean midcenterC = false;
    @Log.BooleanBox(rowIndex = 1, columnIndex = 10, tabName = tabname, width = 1, height = 1 )
   static  boolean midrightC = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 8, tabName = tabname, width = 1, height = 1 )
   static  boolean frontleftC = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 9, tabName = tabname, width = 1, height = 1 )
   static  boolean frontcenterC = false;
    @Log.BooleanBox(rowIndex = 2, columnIndex = 10, tabName = tabname, width = 1, height = 1 )
   static  boolean frontrightC = false;
  TargetSelector(){
  
    row[1] = true;
    column[1] = true;
    grid[1] = true;
    update();

  }

  
  

static void setA(){

    grid[0] = true;
    grid[1] = false;
    grid[2] = false;
    update();

}
static void setB(){

    grid[0] = false;
    grid[1] = true;
    grid[2] = false;
    update();
}
static void setC(){

    grid[0] = false;
    grid[1] = false;
    grid[2] = true;
    update();
}

static void setLeft(){

    column[0] = true;
    column[1] = false;
    column[2] = false;
    update();
} 
static void setCenter(){

    column[0] = false;
    column[1] = true;
    column[2] = false;
    update();
}
static void setRight(){

    column[0] = false;
    column[1] = false;
    column[2] = true;
    update();
}

static void setBack(){

    row[0] = true;
    row[1] = false;
    row[2] = false;
    update();
}
static void setMid(){

    row[0] = false;
    row[1] = true;
    row[2] = false;
    update();
}
static void setFront(){

    row[0] = false;
    row[1] = false;
    row[2] = true;
    update();
}

static int getTargetIdx(){
  //retunrs the index of the target in a flatend array
  //the index reprents targets from left to right the up to down
  //[col][row][grid]
    int targetidx = 0;
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
        for(int k = 0; k < 3; k++){
          if(target[i][j][k]) {
            targetidx = i + j*9 + k*3;
          }
        }
      }
    }
    return targetidx;
}

  
static void update(){

    //create 3d array of booleans to hold the acutalized values of the grid, column, and row
    boolean [][][] gtarget = new boolean[3][3][3];
    boolean [][][] ctarget = new boolean[3][3][3];
    boolean [][][] rtarget = new boolean[3][3][3];
    //[column][row][grid]

    
    // fill the 3d array with the values of the grid, column, and row depending on which is selected for each
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
        for(int k = 0; k < 3; k++){
          gtarget[i][j][k] = grid[k];
          ctarget[i][j][k] = column[i];
          rtarget[i][j][k] = row[j];
        }
      }
    }
    //set the values of the targets to the intersection of the three
    


    //set the values of the targets to the intersection of the three
    //if a target is in all three, it will be true
    backleftA = target[0][0][0] = gtarget[0][0][0] && ctarget[0][0][0] && rtarget[0][0][0];
    backcenterA = target[1][0][0] = gtarget[1][0][0] && ctarget[1][0][0] && rtarget[1][0][0];
    backrightA = target[2][0][0] = gtarget[2][0][0] && ctarget[2][0][0] && rtarget[2][0][0];
    midleftA = target[0][1][0] = gtarget[0][1][0] && ctarget[0][1][0] && rtarget[0][1][0];
    midcenterA = target[1][1][0] = gtarget[1][1][0] && ctarget[1][1][0] && rtarget[1][1][0];
    midrightA = target[2][1][0] = gtarget[2][1][0] && ctarget[2][1][0] && rtarget[2][1][0];
    frontleftA = target[0][2][0] = gtarget[0][2][0] && ctarget[0][2][0] && rtarget[0][2][0];
    frontcenterA = target[1][2][0] = gtarget[1][2][0] && ctarget[1][2][0] && rtarget[1][2][0];
    frontrightA = target[2][2][0] = gtarget[2][2][0] && ctarget[2][2][0] && rtarget[2][2][0];

    backleftB = target[0][0][1] = gtarget[0][0][1] && ctarget[0][0][1] && rtarget[0][0][1];
    backcenterB = target[1][0][1] = gtarget[1][0][1] && ctarget[1][0][1] && rtarget[1][0][1];
    backrightB = target[2][0][1] = gtarget[2][0][1] && ctarget[2][0][1] && rtarget[2][0][1];
    midleftB = target[0][1][1] = gtarget[0][1][1] && ctarget[0][1][1] && rtarget[0][1][1];
    midcenterB = target[1][1][1] = gtarget[1][1][1] && ctarget[1][1][1] && rtarget[1][1][1];
    midrightB = target[2][1][1] = gtarget[2][1][1] && ctarget[2][1][1] && rtarget[2][1][1];
    frontleftB = target[0][2][1] = gtarget[0][2][1] && ctarget[0][2][1] && rtarget[0][2][1];
    frontcenterB = target[1][2][1] = gtarget[1][2][1] && ctarget[1][2][1] && rtarget[1][2][1];
    frontrightB = target[2][2][1] = gtarget[2][2][1] && ctarget[2][2][1] && rtarget[2][2][1];

    backleftC = target[0][0][2] = gtarget[0][0][2] && ctarget[0][0][2] && rtarget[0][0][2];
    backcenterC = target[1][0][2] = gtarget[1][0][2] && ctarget[1][0][2] && rtarget[1][0][2];
    backrightC = target[2][0][2] = gtarget[2][0][2] && ctarget[2][0][2] && rtarget[2][0][2];
    midleftC = target[0][1][2] = gtarget[0][1][2] && ctarget[0][1][2] && rtarget[0][1][2];
    midcenterC = target[1][1][2] = gtarget[1][1][2] && ctarget[1][1][2] && rtarget[1][1][2];
    midrightC = target[2][1][2] = gtarget[2][1][2] && ctarget[2][1][2] && rtarget[2][1][2];
    frontleftC = target[0][2][2] = gtarget[0][2][2] && ctarget[0][2][2] && rtarget[0][2][2];
    frontcenterC = target[1][2][2] = gtarget[1][2][2] && ctarget[1][2][2] && rtarget[1][2][2];
    frontrightC = target[2][2][2] = gtarget[2][2][2] && ctarget[2][2][2] && rtarget[2][2][2];

    

    //[column][row][grid]
    
    




  }
  
  // void clearA(){
  //   backleftA = false;
  //   backcenterA = false;
  //   backrightA = false;
  //   midleftA = false;
  //   midcenterA = false;
  //   midrightA = false;
  //   frontleftA = false;
  //   frontcenterA = false;
  //   frontrightA = false;
  // }
  // void clearB(){
  //   backleftB = false;
  //   backcenterB = false;
  //   backrightB = false;
  //   midleftB = false;
  //   midcenterB = false;
  //   midrightB = false;
  //   frontleftB = false;
  //   frontcenterB = false;
  //   frontrightB = false;
  // }
  // void clearC(){
  //   backleftC = false;
  //   backcenterC = false;
  //   backrightC = false;
  //   midleftC = false;
  //   midcenterC = false;
  //   midrightC = false;
  //   frontleftC = false;
  //   frontcenterC = false;
  //   frontrightC = false;
  // }
  // public boolean newSelection(){
  //   //check if three or less targets are selected
  //   // used to determin if the operator is midway through selecting a target or a new selection
  //   //one of three buttons slect which grid is targeted
  //   //and the other three buttons select which column is targeted
  //   //theese can be chosen in any order, and this function is used to determin if one is already selected and the other needs to only activate the intersection of the two
  //   //or if the operator is starting a new selection
  //   //the row will be selected after and the other selection must come first
  //   //if so, return true
  //   //if not, return false
  //   //im sorry
  //   int targetcount = backleftA ? 1 : backcenterA ? 1 : backrightA ? 1 : midleftA ? 1 : midcenterA ? 1 : midrightA ? 1 : frontleftA ? 1 : frontcenterA ? 1 : frontrightA ? 1 : 
  //   backleftB ? 1 : backcenterB ? 1 : backrightB ? 1 : midleftB ? 1 : midcenterB ? 1 : midrightB ? 1 : frontleftB ? 1 : frontcenterB ? 1 : frontrightB ? 1 : 
  //   backleftC ? 1 : backcenterC ? 1 : backrightC ? 1 : midleftC ? 1 : midcenterC ? 1 : midrightC ? 1 : frontleftC ? 1 : frontcenterC ? 1 : frontrightC ? 1 : 0;
  //   if(targetcount <= 3){
  //     return true;
  //   }

  //   return false;
  // }

}
