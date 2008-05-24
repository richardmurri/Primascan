#! /usr/bin/perl

# simple log reformatter for benoit's sniffusb.exe
# public domain. by anoah@pfeiffer.edu

use strict;

my $count=0;
my $lastUrbLTime=0;
my @buffer;

while (my $line = <STDIN>) {

        if ( $line =~ m/>>>/ ) {
            dumper(@buffer) if $count;
            @buffer = ();
            $count++;
        }
        if ( $line =~ m/UsbSnoop/ ){next;}
        push(@buffer,$line);

}
dumper(@buffer) if $count;

sub dumper {

    my $dir='out';
    my $type='C';
    my $data=();
    my $urb=0;
    my $ep='';
    my $fTime=0;
    my $lTime=0;

    foreach my $line (@_) {

        if($line =~ m/\[(\d+) ms\] .* URB (\d+) going down/){
          $fTime=$1;
          $urb=$2;
          print "\npause " . ($fTime-$lastUrbLTime) . " ms\n\n";
        }
        elsif ( $line =~ m/USBD_TRANSFER_DIRECTION_IN/ ) {
          $dir='in';
          $data=();
        }
        elsif ( $line =~ m/-- URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER/ ) {
          $type='B';
        }
        elsif( $line =~ m/PipeHandle.*endpoint (0x\d+)/){
          $ep=$1;
        }
        elsif ( $line =~ m /  ([0-9a-f]{8}: )([0-9a-f ]*)/ ) {
          push(@{$data}, $1 . $2);
        }

        if ( $line =~ m/\[(\d+) ms\]/ ){
          $lTime=$1;
        }

        $ep=~s/^0x0+//;
    }

    if($dir eq 'in'){
        print "Urb $urb ($type) ep=$ep (read) ";
    }
    else{
        print "Urb $urb ($type) ep=$ep (write) ";
    }
    print $lTime-$fTime . " ms\n";

    foreach my $line (@{$data}) {
            printf(" %s", $line);
            print "\n";
    }

    $lastUrbLTime=$lTime;

}

exit 0;