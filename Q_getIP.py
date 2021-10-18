import netifaces

def getIP():
    interface_list = netifaces.interfaces()
    # Get addresses, netmask, etc. information 
    address_entries = (netifaces.ifaddresses(iface) for iface in interface_list)
    # Only pay attention to ipv4 address types
    ipv4_address_entries = (address[netifaces.AF_INET] for address in address_entries if netifaces.AF_INET in address)
    # Since multiple addresses can be associated, only look at the first ip address
    ipv4_addresses = [address[0]['addr'] for address in ipv4_address_entries]
    if len(ipv4_addresses) >1 :
        ipv4_addresses.remove('127.0.0.1')
        return ipv4_addresses[0]
    else :
        return None

