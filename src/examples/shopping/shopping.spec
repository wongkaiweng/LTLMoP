# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
buyGrocery, 1
buyBook, 1
getMoney, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
MoneyObtained
GroceryDone
BookDone

RegionFile: # Relative path of region description file
shopping.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
getGrocery, 1
getBook, 1
moneyNeeded, 1
GroceryStoreOpen, 1
BookstoreOpen, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
grocery_store = p3
bontique = p5
ATM = p6
bookstore = p4
others = p7, p8, p9
home = p2

Spec: # Specification in structured English
#infinitely often moneyNeeded
# may cause problems at going back home
infinitely often GroceryStoreOpen
# or BookstoreOpen
#if you are sensing getGrocery then do not getBook
#if you are sensing getBook then do not getGrocery
if you were in (bookstore and getBook) or (grocery_store and getGrocery) then do not moneyNeeded

if you were sensing getBook and not BookDone then do getBook
if you were sensing getGrocery and not GroceryDone then do getGrocery

if you are not sensing GroceryStoreOpen then do not grocery_store
#if you are not sensing BookstoreOpen then do not bookstore

##### ATM #######
if you are sensing moneyNeeded and not MoneyObtained then go to ATM
do getMoney if and only if you are sensing ATM and moneyNeeded and not MoneyObtained
MoneyObtained is set on (ATM and getMoney) and reset on (buyGrocery and grocery_store) or (buyBook and bookstore)

### GROCERY STORE #######
if you are sensing MoneyObtained and getGrocery and not GroceryDone then go to grocery_store
do buyGrocery if and only if you are sensing MoneyObtained and getGrocery and grocery_store and not GroceryDone
GroceryDone is set on (buyGrocery and grocery_store) and reset on home

#### BOOKSTORE #####
if you are sensing MoneyObtained and getBook and not BookDone then go to bookstore
do buyBook if and only if you are sensing MoneyObtained and getBook and bookstore and not BookDone
BookDone is set on (buyBook and bookstore) and reset on home

if you are sensing GroceryDone and BookDone then go to home

