#set directory
setwd("G:/Unidades compartidas/GR.INV. PREDATOR-CM/PREDATOR/Papers/1 - Self tuning model predictive control for fluid flows/OA code/utils_LPR")  

# Load all necessary libraries for LPR estimation
library(lokern)

# Clear workspace
rm(list=ls())

#Two different cases:
# 1) Regression function
# 2) Regression function + first derivative

Type <-2
if(Type == 1) {dv <- 0} else{dv <- c(0,1)}
numCases <- length(dv)


# Read data
DataInput    <- read.table("Samples.txt"      , header = FALSE)
EvalPoints   <- read.table("Domain.txt"       , header = FALSE)
EvalPoints   <- as.numeric(unlist(EvalPoints))



# Prepare variables
SizeDataInput <- dim(na.omit(DataInput))
LenDataOutput <- length(na.omit(EvalPoints))
x             <- DataInput[,1]                    # vector of design points
y             <- DataInput[,2:SizeDataInput[2]]   # vector of observation



# Perform LPR estimation of the Regression function (and/or deriv if necessary)
LPR     <-  matrix(nrow = LenDataOutput, ncol = (SizeDataInput[2]-1) * numCases + 1 )
LPR[,1] <-  EvalPoints

for (i in 2:SizeDataInput[2]-1) {
  yi = y[,i]
  est <-lokern::glkerns(x=x,  yi,  x.out=EvalPoints,  deriv=dv[1])
  LPR[,i+1] <-est$est
  
  if(Type == 2) {
    est <-lokern::glkerns(x=x,  yi,  x.out=EvalPoints,  deriv=dv[2])
    LPR[,i+SizeDataInput[2]] <-est$est
  }
}

write.table(LPR, file = "G:/Unidades compartidas/GR.INV. PREDATOR-CM/PREDATOR/Papers/1 - Self tuning model predictive control for fluid flows/OA code/utils_LPR/Output.txt", sep = "\t",
            row.names = FALSE, col.names = FALSE)
