
//------------------------------------------------------------------------------
// Encode onehot signal to binary 
// 
// One hot refers to a group of numbitss among which the legal
// combination of values are only those with a single high (1)
// numbits and all the others are low (0).
//------------------------------------------------------------------------------
module encoder_onehot(
    //--------------------------------------------------------------------------
    // Input interface
    //--------------------------------------------------------------------------
    i__onehot,

    //--------------------------------------------------------------------------
    // Output interface
    //--------------------------------------------------------------------------
    o__valid,
    o__encode
);

//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
parameter NUM_BITS      = 11;
parameter LOG_NUM_BITS  = $clog2(NUM_BITS);
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Input interface
//------------------------------------------------------------------------------
input  logic [NUM_BITS-1:0]             i__onehot;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Output interface
//------------------------------------------------------------------------------
output logic                            o__valid;
output logic [LOG_NUM_BITS-1:0]         o__encode;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Conversion logic
//------------------------------------------------------------------------------
generate
if(NUM_BITS == 2)
begin: num_bits2
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            2'b01:      begin o__valid = 1'b1; o__encode = 1'b0; end
            2'b10:      begin o__valid = 1'b1; o__encode = 1'b1; end
            default:    begin o__valid = 1'b0; o__encode = 'bx; end
        endcase
    end
end
else if(NUM_BITS == 4)
begin: num_bits4
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            4'b0001:    begin o__valid = 1'b1; o__encode = 2'b00; end
            4'b0010:    begin o__valid = 1'b1; o__encode = 2'b01; end
            4'b0100:    begin o__valid = 1'b1; o__encode = 2'b10; end
            4'b1000:    begin o__valid = 1'b1; o__encode = 2'b11; end
            default:    begin o__valid = 1'b0; o__encode = 'bx; end
        endcase
    end
end
else if(NUM_BITS == 6)
begin: num_bits6
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            6'b000001:    begin o__valid = 1'b1; o__encode = 3'b000; end
            6'b000010:    begin o__valid = 1'b1; o__encode = 3'b001; end
            6'b000100:    begin o__valid = 1'b1; o__encode = 3'b010; end
            6'b001000:    begin o__valid = 1'b1; o__encode = 3'b011; end
            6'b010000:    begin o__valid = 1'b1; o__encode = 3'b100; end
            6'b100000:    begin o__valid = 1'b1; o__encode = 3'b101; end
            default:      begin o__valid = 1'b0; o__encode = 3'b111; end
        endcase
    end
end
else if(NUM_BITS == 7)
begin: num_bits6
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            7'b0000001:    begin o__valid = 1'b1; o__encode = 3'b000; end
            7'b0000010:    begin o__valid = 1'b1; o__encode = 3'b001; end
            7'b0000100:    begin o__valid = 1'b1; o__encode = 3'b010; end
            7'b0001000:    begin o__valid = 1'b1; o__encode = 3'b011; end
            7'b0010000:    begin o__valid = 1'b1; o__encode = 3'b100; end
            7'b0100000:    begin o__valid = 1'b1; o__encode = 3'b101; end
            7'b1000000:    begin o__valid = 1'b1; o__encode = 3'b110; end
            default:      begin o__valid = 1'b0; o__encode = 3'b111; end
        endcase
    end
end
else if(NUM_BITS == 8)
begin: num_bits8
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            8'b00000001:    begin o__valid = 1'b1; o__encode = 3'b000; end
            8'b00000010:    begin o__valid = 1'b1; o__encode = 3'b001; end
            8'b00000100:    begin o__valid = 1'b1; o__encode = 3'b010; end
            8'b00001000:    begin o__valid = 1'b1; o__encode = 3'b011; end
            8'b00010000:    begin o__valid = 1'b1; o__encode = 3'b100; end
            8'b00100000:    begin o__valid = 1'b1; o__encode = 3'b101; end
            8'b01000000:    begin o__valid = 1'b1; o__encode = 3'b110; end
            8'b10000000:    begin o__valid = 1'b1; o__encode = 3'b111; end
            default:        begin o__valid = 1'b0; o__encode = 'bx; end
        endcase
    end
end
else if(NUM_BITS == 1)
begin: num_bits11
    always_comb
    begin
        case(i__onehot) // synopsys full_case parallel_case
            11'b00000000001:    begin o__valid = 1'b1; o__encode = 4'b0000; end
            11'b00000000010:    begin o__valid = 1'b1; o__encode = 4'b0001; end
            11'b00000000100:    begin o__valid = 1'b1; o__encode = 4'b0010; end
            11'b00000001000:    begin o__valid = 1'b1; o__encode = 4'b0011; end
            11'b00000010000:    begin o__valid = 1'b1; o__encode = 4'b0100; end
            11'b00000100000:    begin o__valid = 1'b1; o__encode = 4'b0101; end
            11'b00001000000:    begin o__valid = 1'b1; o__encode = 4'b0110; end
            11'b00010000000:    begin o__valid = 1'b1; o__encode = 4'b0111; end
            11'b00100000000:    begin o__valid = 1'b1; o__encode = 4'b1000; end
            11'b01000000000:    begin o__valid = 1'b1; o__encode = 4'b1001; end
            11'b10000000000:    begin o__valid = 1'b1; o__encode = 4'b1010; end
            default:        begin o__valid = 1'b0; o__encode = 'bx; end
        endcase
    end
end
else
begin: num_bits_other
    always_comb
    begin
        o__valid    = 1'b0;
    
        // Don't output Xs in RTL simulation. vcs treats x's as a separate entity
        // from 0 and 1, leading to bugs
        // For e.g. (data_out == 0) will always be false in vcs if data_out is x,
        // but could become true in a real simulation.
    
        o__encode = 'bx;
    
        for(int i = 0; i < NUM_BITS; i++)
        begin
            if(i__onehot == ($unsigned(1) << $unsigned(i)))
            begin
                o__valid    = 1'b1;
                o__encode   = $unsigned(i);
                break;
            end
        end
    end
end
endgenerate
//------------------------------------------------------------------------------
    
endmodule   // encoder_onehot

