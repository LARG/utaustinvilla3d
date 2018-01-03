#!/usr/bin/perl

# kickoff.pl
# Patrick MacAlpine (adapted from BATS code)
# patmac@cs.utexas.edu
# 2009-11-27
#
# This perl script will start a kickoff for the left team
#   kickoff.pl
#
# Do a kickoff-left, and then quit.

use IO::Socket;
use Math::Trig;

sub prepareMsg
{
  my $msg = shift;
  my $len = length($msg);
  my $pref = pack 'N',$len;
  return $pref.$msg;
}

my $host = 'localhost';
my $port = $ENV{'SPARK_SERVERPORT'} || 3200;
my $proto = getprotobyname("tcp");

my $iaddr = inet_aton($host);
my $paddr = sockaddr_in($port, $iaddr);

$/ = "\0";

print "Connecting to server $host:$port...\n";

my $sock = new IO::Socket::INET ( PeerAddr => $host, PeerPort => $port, Proto => 'tcp',  );
die "Could not create socket: $!\n" unless $sock;

print "Kicking off\n";
print $sock prepareMsg("(playMode KickOff_Left)");
<$sock>;

close($sock) or die "Failed closing socket: $!\n";
