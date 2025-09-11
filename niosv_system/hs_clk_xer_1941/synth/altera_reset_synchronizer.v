// (C) 2001-2025 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/25.1.1/ip/iconnect/merlin/altera_reset_controller/altera_reset_synchronizer.v#1 $
// $Revision: #1 $
// $Date: 2025/04/24 $

// -----------------------------------------------
// Reset Synchronizer
// -----------------------------------------------
`timescale 1 ns / 1 ns

module altera_reset_synchronizer
#(
    parameter ASYNC_RESET = 1,
    parameter DEPTH       = 2
)
(
    input   reset_in /* synthesis ALTERA_ATTRIBUTE = "SUPPRESS_DA_RULE_INTERNAL=R101" */,

    input   clk,
    output  reset_out
);

    // -----------------------------------------------
    // Synchronizer register chain. We cannot reuse the
    // standard synchronizer in this implementation 
    // because our timing constraints are different.
    //
    // Instead of cutting the timing path to the d-input 
    // on the first flop we need to cut the aclr input.
    // 
    // We omit the "preserve" attribute on the final
    // output register, so that the synthesis tool can
    // duplicate it where needed.
    // -----------------------------------------------
    (*preserve*) reg [DEPTH-1:0] altera_reset_synchronizer_int_chain;
    reg altera_reset_synchronizer_int_chain_out;

    generate if (ASYNC_RESET) begin

        // -----------------------------------------------
        // Assert asynchronously, deassert synchronously.
        // -----------------------------------------------
        always @(posedge clk or posedge reset_in) begin
            if (reset_in) begin
                altera_reset_synchronizer_int_chain <= {DEPTH{1'b1}};
                altera_reset_synchronizer_int_chain_out <= 1'b1;
            end
            else begin
                altera_reset_synchronizer_int_chain[DEPTH-2:0] <= altera_reset_synchronizer_int_chain[DEPTH-1:1];
                altera_reset_synchronizer_int_chain[DEPTH-1] <= 0;
                altera_reset_synchronizer_int_chain_out <= altera_reset_synchronizer_int_chain[0];
            end
        end

        assign reset_out = altera_reset_synchronizer_int_chain_out;
     
    end else begin

        // -----------------------------------------------
        // Assert synchronously, deassert synchronously.
        // -----------------------------------------------
        always @(posedge clk) begin
            altera_reset_synchronizer_int_chain[DEPTH-2:0] <= altera_reset_synchronizer_int_chain[DEPTH-1:1];
            altera_reset_synchronizer_int_chain[DEPTH-1] <= reset_in;
            altera_reset_synchronizer_int_chain_out <= altera_reset_synchronizer_int_chain[0];
        end

        assign reset_out = altera_reset_synchronizer_int_chain_out;
 
    end
    endgenerate

endmodule

`ifdef QUESTA_INTEL_OEM
`pragma questa_oem_00 "2drP+sazgX0dFgvJXkad//JE7EoPB7Z9a/l/nmbWjjvOJ1vTyAej+27pfC9dUizmcVmwGHLzebopS/IoT2mM0rDIBTeOSaYKTz3KdEH/h9XSkHdjtfp4aGXYu6ARXLVobymCLGNQ20gkA0gDGGsYTHUMlIaJZSSNKBSNue4fOjBo+OUtfL1cAmu3nXseSTP9lXqepLK9DxyLdom6TcCWNQ1dGjNWd/UaIN8XTqFQX9fUjvffcsA+PehrmksKmnaBgqeOz1d/avH8mCK0mgL91qvbZEiZdMfbNweO604Tae5T66IOAcDwrRgX7Zt1WVTQYlJCDOxsRmaf9PeU5gqVrCmN8w+2hjmbve7XyKoT+HnKCEn/kDwoltjys4l4OUkqSc9AR6teIrg9u230oYJzEHJYJwrcauEpDyrAN6zGJlFy0tzip/Ecw5YM2qknkSuPtpY2M9wEzsKHPwUQn9kmAQJXYq2rzWByT64PNIu4kRhRfjXjN1ztkpQ2/Kj0RlqVht8yGUswSgO1PaAJX0WH27JIX/RR0xkvJzq8yHddljCrZZKzviM1HBk0VxecWVI0myM2935YepnZsSM4c5/RbjZ5mAh+lFfCcBc9yanauvDpmqoG8dJF7NRF3ox6+MFaaqlneNNM+BJEamrF04LjeaM6gyEl1JzHpybX4d6zQApqroVnUmAfevmNmAXZeKPY4CxLMzN/J6rA9v3Xyq/CEJYxot7X7Hntr9FgHFTBu5TJx1OoqTuDEzCzdDcsC7toKwsor0yH8pCk7fISiaonqvUKvhNqaFk8vphKQn62RtvgAON4ie2UTILYlBxQcuXgDaGeSW+7tOUute5WKei1KJGPk0k4ik/u4ZgVW8hGSQUPeJyrd9O6lvD6WrIKuNGNTNMOSTX5LbijolWuwju0tMUVIV4y6vQDiYcjkhpgHyfL7cEXYBnKUojz7zl2Y2OHHYUF60Vh3AyntGf6CtzmyV1Q9FUsbY7XN0FCzXjHbBaIkx+EPLiaDYHCtA4j2F4A"
`endif