#!/usr/bin/perl

$prj = $ARGV[0];
$cust = "";
$cust = $ARGV[1] if($#ARGV == 1);


$custdir = "./wingcust/${prj}";
$ConfigPath = "./mediatek/config/${prj}";
$CustomPath = "./mediatek/custom/${prj}";
$VendorPath = "./vendor/mediatek/${prj}";
$VendorFile = "$custdir/base/vendor/config.ini";
$GMSPath = "./vendor";

print "prj=${prj}, cust=${cust}\n";


#copy customers's  config to mediatk/config
if(-e  "${ConfigPath}")
{
   system("rm -rf ${ConfigPath}");
}
system("cp -a ${custdir}/base/config ${ConfigPath}");
if ($cust ne "")
{
    system("cp -a ${custdir}/${cust}/config/* ${ConfigPath}") if (-e "${custdir}/${cust}/config");
}

#copy customers's custom to /mediatek/custom
if(-e "${CustomPath}")
{
   system("rm -rf ${CustomPath}");
}
system("cp -a ${custdir}/base/custom  ${CustomPath}");
if ($cust ne "")
{
    system("cp -a ${custdir}/${cust}/custom/* ${CustomPath}") if (-e "${custdir}/${cust}/custom");
}

#copy customers's gms to /vendor/google
if(-e "${GMSPath}/google")
{
   system("rm -rf ${GMSPath}/google");
}
if (-e "${custdir}/base/gms")
{
    system("cp -a ./vendor/gms/google  ${GMSPath}");
    system("cp -a ${custdir}/base/gms/google  ${GMSPath}");
}
if ($cust ne "")
{
    system("cp -a ./vendor/gms/google  ${GMSPath}");
    system("cp -a ${custdir}/${cust}/gms/google ${GMSPath}") if (-e "${custdir}/${cust}/gms/google");
}


#copy platform's vendor to ./vendor/mediatek/${project} by config.ini at wingcust/${prj}/base/vendor dir
if(-e "${VendorPath}")
{
   system("rm -rf ${VendorPath}");
}
system("mkdir ${VendorPath}");
if(-e "$VendorFile")
{
	  my @del_list;
    open( V_FILE, "<$VendorFile") or die "can not open $VendorFile:$!\n";

    while(<V_FILE>)
    {
	      chomp;
	      if (/^\s*parents\s*=\s*(\S+)/) 
	      {
	      	$parentsProject = $1;
	      }
	      if (/^\s*del\s*=\s*(\S+)/) 
	      {
	      	push @del_list,$1;
	      }  
    }
    close V_FILE;

    die "Project have vendor config.ini but parentsProject NOT defined!"
        if(!$parentsProject);
    die "${parentsProject} not exist!"
        if(!-d "./vendor/mediatek/${parentsProject}");  
 
	  #copy parents dir 
    system("cp -a \\./vendor/mediatek/${parentsProject}/*  ${VendorPath}");
    system("mv  ${VendorPath}/artifacts/out/target/product/${parentsProject}  ${VendorPath}/artifacts/out/target/product/${prj}");
	
	  # del file not need
    foreach my $delfile (@del_list) {
        system("rm -rvf ${VendorPath}/${delfile}");
    }
	
}
#copy customers's vendor to ./vendor/mediatek/${project}
{
    system("cp -a ${custdir}/base/vendor/*  ${VendorPath}");
    if ($cust ne "")
    {
	  	  system("cp -a ${custdir}/${cust}/vendor/* ${VendorPath}") if (-e "${custdir}/${cust}/vendor");
    }
}

#copy lk&product ${project}.mk
system("cp -f ${custdir}/product/${prj}.mk  ./build/target/product/${prj}.mk");
if(-e "${custdir}/${cust}/product/${prj}.mk")
{
	system("cp -f ${custdir}/${cust}/product/${prj}.mk  ./build/target/product/${prj}.mk");
}
if(-e "${custdir}/${cust}/gms/google/products/gms.mk")
{
	system("cp -f ${custdir}/${cust}/gms/google/products/gms.mk  ${GMSPath}/google/products/gms.mk");
}

system("cp -f ${custdir}/lk/${prj}.mk  ./bootable/bootloader/lk/project/${prj}.mk");
#add for lk lcm  ask for jinzengwen
system("cp -a ${CustomPath}/kernel/lcm/ ${CustomPath}/lk/lcm") if (-e "${CustomPath}/kernel/lcm");



#sunxuewen added begin for huawei-y360
if(-e "./wingcust/${prj}/${cust}/default_area.txt")
{
	system("rm -f ./vendor/huawei/default_area.txt");
	system("cp  ./wingcust/${prj}/${cust}/default_area.txt ./vendor/huawei/");
}
#sunxuewen added end
exit 0;
