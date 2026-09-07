# Security

Report vulnerabilities to hello@flightportrait.com. A human reads
that address. Do not open a public issue for a report that could
harm feeders.

rx listens on the LAN ports it is given (Beast output, Beast input)
and opens outbound connections to the aggregators in its
`--net-connector` list. It stores no keys of its own: the station
UUID is passed on the command line by stationd and sent to the
aggregators as the connector hello. It never stores or forwards the
addresses of the machines that connect to it.
