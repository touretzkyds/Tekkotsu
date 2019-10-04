#!/usr/bin/perl -w
use strict;

###############################################################################
#
#  Author: Stuart Seymon
#  Mi-Pal Griffith University Australia
#
#  This program is copyrighted to the Author and released under the GPL(version2).
#  See doc/license.txt
#
#  elp.pl   parse the emon.log error file
#
#  Usage
#
#  ./epl.pl [-e emonFile][-A]
#
#  -e specifies the EMON.LOG file to be "emonFile"
#     default emonFile: ./EMON.LOG
#  -A stop assembly code being printed to stdout
#
#  
#  This script will print out all the relevant information to be ascertained 
#  from the EMON.LOG file that is produced whenever the AIBO crashes.  It will 
#  use this information to determine the assembly code address that caused the
#  exception.  If '-A' is not used, it will print out a section of the assembly
#  code, as well as the important aspects of EMON.LOG.  
#
#  Regardless of the 'A' switch being used, it will print out the entire 
#  assembly code to a file in the cwd labelled: 
#  aiboDis<n>.ass  
#  where:
#  n is the number of disassemblies residing in the cwd
#  
#  N.B.  the *ass files by their nature are quite large, so be sure to 
#  remove your .ass files when you are finished with them.

my $TRUE    = 0;
my $FALSE   = 1;

my $line;
my @lines;
my @output;
my @throwaway;
my $continue;

my $excepDescrip;       
my $objContextId;       
my $excepObj;           
my $BadVAddr;
my $epc;
my $gpVal;
my $objDir;
my $addr_gp;
my $ra;
my $linkTimeAddr;

my $showAssembly = $TRUE;
my $assemFile;
my $timeStamp = time();

my $crashLine;

my $lineCounter    = 0;
my $genericCounter = 0;
my $blah;
my @stuff;                 #used to split a line into its individual pieces

my $emonFile   = "./EMON.LOG";
my $PREFIX     = "aiboDis";
my $EXTENSION  = "ass";

my $assemCount;   #the number of disassems in cwd
my $ls         = "ls";
my $wc         = "wc -l";
my $OPENRSDK_ROOT = "/usr/local/OPEN_R_SDK";        #can be overridden by environment variable of the same name
if( defined $ENV{"OPENRSDK_ROOT"} ) {
	$OPENRSDK_ROOT = $ENV{"OPENRSDK_ROOT"};
}
my $mlr        = "$OPENRSDK_ROOT/bin/mipsel-linux-readelf -s";    #relative to OPENRSDK_ROOT
my $mlo        = "$OPENRSDK_ROOT/bin/mipsel-linux-objdump -S -C"; #relative to OPENRSDK_ROOT

my $OBJ_NAME_POS     = 0;  #pos of each value in each relevant line
my $BADVADDR_POS     = 5;  
my $EPC_POS          = 9;
my $OB_CONT_ID_POS   = 2;  
my $GPVAL_POS        = 5;
my $RA_POS           = 2;

my $MAX_ASSEMBLY_OUTPUT = 100;   

select(STDOUT);
$| = 1;

@stuff = split(/^\s*/, `$ls $PREFIX*|$wc`);
$assemCount = $stuff[1];   
chomp $assemCount;

$assemFile = "$PREFIX$assemCount.$EXTENSION";


#get command line arguments
foreach(@ARGV)
{
   if($ARGV[$genericCounter] eq "-e")
   {
      if(($genericCounter + 1) < scalar@ARGV)
      {
         $emonFile = $ARGV[$genericCounter + 1];
      }
      else 
      {
         &usage();
         exit(1);
      }
   }
   elsif($ARGV[$genericCounter] eq "-A")
   {
      $showAssembly = $FALSE;
   }
   elsif($ARGV[$genericCounter] eq "-help")
   {
      &usage();
      exit(1);
   }
    
   $genericCounter++;
}

#############################################################################
#  
#  parse EMON.LOG
#
open (EMON, $emonFile) || die "Cannot open $emonFile";
while ($line = <EMON>)
{
   chomp($line);
   push(@lines, $line);
   
   if($objContextId)
   {
      if ($line =~/\Q$objContextId/)
      {
         @stuff = split(/\s+/, $line);
         $excepObj = $stuff[$OBJ_NAME_POS];
      }
   }
   
   if($line =~/^exception code/)
   {
      $excepDescrip = $line;
   }
   
   if($line =~/badvaddr.*epc/)
   {
      @stuff      = split(/\s+|:/, $line);
      $BadVAddr   = $stuff[$BADVADDR_POS];
      ($BadVAddr, @throwaway) = split(/,/, $BadVAddr);
      $epc        = $stuff[$EPC_POS];
   }
   
   if($line =~/ra:r31/)
   {
      @stuff = split(/\s+|,/, $line);
      $ra    = $stuff[$RA_POS];
   }   
      
   if($line =~/gp:r28/)
   {
      @stuff = split(/\s+|,/, $line);
      $gpVal = $stuff[$GPVAL_POS];
   }

   #get the object context_id that caused the exception
   if($line =~/context.*state/)
   {
      @stuff = split(/\s+|,/, $line);
      $objContextId = $stuff[$OB_CONT_ID_POS];
   }
   
      
   $lineCounter++;
}
close EMON;

#############################################################################
#
#  error checking and analysis of grabbed data
#
if(!$excepObj)  
{
   print "Unable to match context address with object\n";
   exit(1);
}

if($excepDescrip =~/ddress/)
{  
   push(@output, ("Address that failed to have a valid reference:\n\t" . $BadVAddr));
}
elsif($excepDescrip =~/TLB/)
{
   push(@output, ("Virtual address that failed to have a valid translation:\n\t\t\t" . $BadVAddr));
}

if($BadVAddr eq $epc)
{
   push(@output, ("Jumped to an invalid address"));
}


#############################################################################
#
#  get the address of _gp using mipsel-linux-readelf
#
$blah = $excepObj . ".nosnap.elf|grep \'_gp\$\'";
@stuff = `$mlr $blah`;

if(!@stuff)          #object was not made in cwd
{
   print "\nPlease specify the directory where $excepObj was made:\n";
   $objDir = <STDIN>;
   chomp $objDir;
      
   if(!($objDir =~/\/$/))
   {
      $objDir = $objDir . "/";
   }

   $blah = $objDir . $excepObj . ".nosnap.elf |grep \'_gp\$\'";
   @stuff = `$mlr $blah`;
   
   if(!@stuff)       #still wrong
   {
      die "unable to find $excepObj.nosnap.elf in $objDir\n";
   }
}

@stuff = split(/ /, $stuff[0]);

$genericCounter = 0;
while($genericCounter < scalar@stuff)
{
   if($stuff[$genericCounter] =~/[\dabcdef]{8}/)
   {
      $addr_gp = "0x". $stuff[$genericCounter];
   }
   $genericCounter++;
} 



############################################################################
#
#  Do the hex addition/subtraction
#

$linkTimeAddr = hex($epc) - hex($gpVal);
$linkTimeAddr += hex($addr_gp);
$linkTimeAddr = sprintf("%x", $linkTimeAddr);


############################################################################         
#                                                                                    
#  Time to look at the assembly code                                                 
if ($objDir)                                                                         
{
   $blah = $objDir . $excepObj . ".nosnap.elf";
}
else
{
   $blah = $excepObj . ".nosnap.elf";
}

@stuff = `$mlo $blah`;     #the output of this is quite large, and thus
                           #takes a while to execute

open (BLAH, ">$assemFile");

print BLAH $excepDescrip;                                
print BLAH "\nObject Name:        " . $excepObj;         
print BLAH "\nObject Context ID:  " . $objContextId;     
print BLAH "\nBadVAddr:           " . $BadVAddr;         
print BLAH "\nEPC:                " . $epc;
print BLAH "\ngpVal               " . $gpVal;
print BLAH "\nLink Time Address:  " . $linkTimeAddr;   
print BLAH "\naddr_gp:            " . $addr_gp;
print BLAH "\nra:                 " . $ra;
print BLAH "\n\n";                                       

$genericCounter = 0;
while($genericCounter < scalar@stuff)
{  
   if($genericCounter % 1000 == 0)
   {
      #print ".";
      #$! = 1;
   }
   print BLAH $stuff[$genericCounter];
   chomp($stuff[$genericCounter]);
   if($stuff[$genericCounter] =~/^(\s{0,4})?\Q$linkTimeAddr/)
   {
      $crashLine = $genericCounter;
   }
   $genericCounter++;
}
print "\n";
close (BLAH);

if(!$crashLine)
{
   &printResults(@output);
   die "unable to find link time address in assembly code\n";
}

$genericCounter = $crashLine;
while(!($stuff[$genericCounter] =~ /^\s*$/))
{
   $genericCounter--;
}

$continue = $TRUE;
if($showAssembly == $TRUE)
{
   $blah = $genericCounter;
   while (($genericCounter < ($crashLine + 10)) && ($continue == $TRUE))
   {
      if($genericCounter > $crashLine)
      {
         if($stuff[$genericCounter] =~ /^\s*$/)
         {
            $continue = $FALSE;
         }
      }
      push(@output, $stuff[$genericCounter]);

      if($genericCounter == $crashLine)
      {
         push(@output, "  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
      }
      
      $genericCounter++;
   }
   if($genericCounter - $blah > $MAX_ASSEMBLY_OUTPUT)
   {
      push(@output, "\nExcessive ouput detected: use \'-A\' to stop assembly output");
      push(@output, "Entire assembly code has been written to file, \"$assemFile\"\n");
   }
}

push(@output, $excepDescrip);
push(@output, ("\nObject Name: \t\t" . $excepObj));
push(@output, ("object Context id: \t" . $objContextId));
push(@output, ("BadVAddr:\t\t" . $BadVAddr));
push(@output, ("EPC: \t\t\t" . $epc));
push(@output, ("gpVal: \t\t\t" . $gpVal));
push(@output, ("linkTimeAddr: \t\t  0x" . $linkTimeAddr));
push(@output, ("addr_gp: \t\t" . $addr_gp));
push(@output, ("ra: \t\t\t" . $ra));

print "\n";

&printResults(@output);

####### END script




sub printResults
{
   my $count = 0;
   
   while ($count < scalar@_)
   {
      print $_[$count++] . "\n";
   }
}

sub usage
{
   print "usage:\n";
   print "\n./elp.pl [-e emonFile][-A]\n\n";
   print "-e specify the location of EMON.LOG to be <emonFile>\n";
   print "   default: ./EMON.LOG\n";
   print "-A stops the output of assembly code to stdout.\n";
}
